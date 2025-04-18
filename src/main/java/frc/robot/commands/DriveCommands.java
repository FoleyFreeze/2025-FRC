// Copyright 2021-2025 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.commands;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.subsystems.drive.Drive;
import frc.robot.util.Locations;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.LinkedList;
import java.util.List;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class DriveCommands {
    private static final double DEADBAND = 0.1;
    private static final double ANGLE_KP = 5.0;
    private static final double ANGLE_KD = 0.4;
    private static final double ANGLE_MAX_VELOCITY = 8.0;
    private static final double ANGLE_MAX_ACCELERATION = 20.0;
    private static final double FF_START_DELAY = 1.0; // Secs
    private static final double FF_RAMP_RATE = 0.2; // Volts/Sec
    private static final double WHEEL_RADIUS_MAX_VELOCITY = 0.25; // Rad/Sec
    private static final double WHEEL_RADIUS_RAMP_RATE = 0.05; // Rad/Sec^2

    private static final double POS_KP = 8;
    private static final double POS_KI = 0;
    private static final double POS_KD = 0;
    private static final double POS_MAX_VEL = 1; // m/s
    private static final double POS_TOL = Units.inchesToMeters(0.5);
    private static final double POS_MAX_DIST =
            Units.inchesToMeters(10); // dont drive if more than 10in away
    private static final double POS_MAX_TIME = 0.75;
    private static final double POS_TOL_GATHER = Units.inchesToMeters(2.0);
    private static final double POS_TOL_SCORE = Units.inchesToMeters(0.5);

    private DriveCommands() {}

    private static Translation2d getLinearVelocityFromJoysticks(double x, double y) {
        // Apply deadband
        double linearMagnitude = MathUtil.applyDeadband(Math.hypot(x, y), DEADBAND);
        Rotation2d linearDirection = new Rotation2d(Math.atan2(y, x));

        // Square magnitude for more precise control
        linearMagnitude = linearMagnitude * linearMagnitude;

        // Return new linear velocity
        return new Pose2d(new Translation2d(), linearDirection)
                .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
                .getTranslation();
    }

    public static Command joystickDriveFlysky(RobotContainer r) {
        return joystickDrive(
                r,
                () -> -r.flysky.getLeftY(),
                () -> -r.flysky.getLeftX(),
                () -> -r.flysky.getRightX());
    }

    /**
     * Field relative drive command using two joysticks (controlling linear and angular velocities).
     */
    public static Command joystickDrive(
            RobotContainer r,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier) {
        return Commands.run(
                () -> {
                    // Get linear velocity
                    Translation2d linearVelocity =
                            getLinearVelocityFromJoysticks(
                                    xSupplier.getAsDouble(), ySupplier.getAsDouble());

                    // Apply rotation deadband
                    double omega = MathUtil.applyDeadband(omegaSupplier.getAsDouble(), DEADBAND);

                    // Square rotation value for more precise control
                    omega = Math.copySign(omega * omega, omega);

                    // Convert to field relative speeds & send command
                    ChassisSpeeds speeds =
                            new ChassisSpeeds(
                                    linearVelocity.getX() * r.drive.getMaxLinearSpeedMetersPerSec(),
                                    linearVelocity.getY() * r.drive.getMaxLinearSpeedMetersPerSec(),
                                    omega * r.drive.getMaxAngularSpeedRadPerSec());

                    // "turbo" button
                    if (r.elevator.getHeight().in(Inches) > 5) {
                        speeds = speeds.times(0.5);
                    }

                    boolean isFlipped =
                            DriverStation.getAlliance().isPresent()
                                    && DriverStation.getAlliance().get() == Alliance.Red;
                    r.drive.runVelocity(
                            ChassisSpeeds.fromFieldRelativeSpeeds(
                                    speeds,
                                    isFlipped
                                            ? r.drive.getRotation().plus(new Rotation2d(Math.PI))
                                            : r.drive.getRotation()));
                },
                r.drive);
    }

    /**
     * Field relative drive command using joystick for linear control and PID for angular control.
     * Possible use cases include snapping to an angle, aiming at a vision target, or controlling
     * absolute rotation with a joystick.
     */
    public static Command joystickDriveAtAngle(
            RobotContainer r,
            DoubleSupplier xSupplier,
            DoubleSupplier ySupplier,
            DoubleSupplier omegaSupplier,
            Supplier<Rotation2d> rotationSupplier) {

        // Create PID controller
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP,
                        0.0,
                        ANGLE_KD,
                        new TrapezoidProfile.Constraints(
                                ANGLE_MAX_VELOCITY, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(2));

        // evil hack to be effectively final
        final double[] lastManualTime = new double[1];
        lastManualTime[0] = 0;
        double manualTimeThresh = 0.35;
        double manualThresh = 0.1;

        // Construct command
        return Commands.run(
                        () -> {
                            // Get linear velocity
                            Translation2d linearVelocity =
                                    getLinearVelocityFromJoysticks(
                                            xSupplier.getAsDouble(), ySupplier.getAsDouble());

                            // determine mode
                            if (Math.abs(omegaSupplier.getAsDouble()) > manualThresh) {
                                lastManualTime[0] = Timer.getTimestamp();
                            }

                            // Calculate angular speed
                            double omega;
                            if (Timer.getTimestamp() - lastManualTime[0] > manualTimeThresh) {
                                // in pid mode
                                omega =
                                        angleController.calculate(
                                                r.drive.getRotation().getRadians(),
                                                rotationSupplier.get().getRadians());
                                if (angleController.atGoal()) {
                                    omega = 0;
                                }

                            } else {
                                // in manual mode
                                omega =
                                        omegaSupplier.getAsDouble()
                                                * r.drive.getMaxAngularSpeedRadPerSec();
                            }

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds =
                                    new ChassisSpeeds(
                                            linearVelocity.getX()
                                                    * r.drive.getMaxLinearSpeedMetersPerSec(),
                                            linearVelocity.getY()
                                                    * r.drive.getMaxLinearSpeedMetersPerSec(),
                                            omega);

                            if (r.elevator.getHeight().in(Inches) > 3) {
                                speeds = speeds.times(0.35);
                            } else if (r.flysky.leftTriggerSWE.getAsBoolean()) {
                                speeds = speeds.times(0.5);
                            }

                            boolean isFlipped =
                                    DriverStation.getAlliance().isPresent()
                                            && DriverStation.getAlliance().get() == Alliance.Red;
                            r.drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            speeds,
                                            isFlipped
                                                    ? r.drive
                                                            .getRotation()
                                                            .plus(new Rotation2d(Math.PI))
                                                    : r.drive.getRotation()));
                        },
                        r.drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(r.drive.getRotation().getRadians()));
    }

    public static Command driveToAngle(RobotContainer r, Supplier<Rotation2d> angle) {
        // Create PID controller
        ProfiledPIDController angleController =
                new ProfiledPIDController(
                        ANGLE_KP,
                        0.0,
                        ANGLE_KD,
                        new TrapezoidProfile.Constraints(2, ANGLE_MAX_ACCELERATION));
        angleController.enableContinuousInput(-Math.PI, Math.PI);
        angleController.setTolerance(Units.degreesToRadians(2));

        // Construct command
        return Commands.run(
                        () -> {
                            // Calculate angular speed
                            double omega;
                            // in pid mode
                            omega =
                                    angleController.calculate(
                                            r.drive.getRotation().getRadians(),
                                            angle.get().getRadians());
                            if (angleController.atGoal()) {
                                omega = 0;
                            }

                            // Convert to field relative speeds & send command
                            ChassisSpeeds speeds = new ChassisSpeeds(0, 0, omega);

                            r.drive.runVelocity(speeds);
                        },
                        r.drive)

                // Reset PID controller when command starts
                .beforeStarting(() -> angleController.reset(r.drive.getRotation().getRadians()));
    }

    public static Command driveToPoint(
            RobotContainer r, Supplier<Pose2d> supplier, boolean isGather) {
        PIDController pidX = new PIDController(POS_KP, POS_KI, POS_KD);
        PIDController pidY = new PIDController(POS_KP, POS_KI, POS_KD);
        final double POS_TOL;
        if (isGather) {
            POS_TOL = POS_TOL_GATHER;
        } else {
            POS_TOL = POS_TOL_SCORE;
        }
        // pidX.setTolerance(POS_TOL);
        // pidY.setTolerance(POS_TOL);

        double[] error = new double[1];
        Timer timer = new Timer();

        return Commands.run(
                        () -> {
                            Pose2d target = supplier.get();
                            Pose2d meas = r.drive.chooseLocalPose();

                            Translation2d pointErr =
                                    target.getTranslation().minus(meas.getTranslation());
                            error[0] = pointErr.getNorm();
                            Logger.recordOutput("Odometry/PointErr", pointErr);
                            Logger.recordOutput("Odometry/PointErrNorm", error[0]);

                            double xVel = pidX.calculate(meas.getX(), target.getX());
                            double yVel = pidY.calculate(meas.getY(), target.getY());

                            xVel = MathUtil.clamp(xVel, -POS_MAX_VEL, POS_MAX_VEL);
                            yVel = MathUtil.clamp(yVel, -POS_MAX_VEL, POS_MAX_VEL);

                            // TODO: run angle PID in parallel
                            r.drive.runVelocity(
                                    ChassisSpeeds.fromFieldRelativeSpeeds(
                                            new ChassisSpeeds(xVel, yVel, 0),
                                            r.drive.getRotation()));

                            // only enable autoshoot if we got the right spot and there is an april
                            // tag confirming it
                            if (error[0] < POS_TOL) {
                                r.state.onTarget = r.vision.selectedTagOnTarget();
                                // r.state.onTarget = true;
                            }
                        },
                        r.drive)
                .until(() -> error[0] < POS_TOL || timer.hasElapsed(POS_MAX_TIME))
                .beforeStarting(
                        () -> {
                            pidX.reset();
                            pidY.reset();
                            timer.restart();
                            r.state.onTarget = false;
                        })
                .andThen(
                        new InstantCommand(
                                () -> r.drive.runVelocity(new ChassisSpeeds()), r.drive));
    }

    /**
     * Measures the velocity feedforward constants for the drive motors.
     *
     * <p>This command should only be used in voltage control mode.
     */
    public static Command feedforwardCharacterization(Drive drive) {
        List<Double> velocitySamples = new LinkedList<>();
        List<Double> voltageSamples = new LinkedList<>();
        Timer timer = new Timer();

        return Commands.sequence(
                // Reset data
                Commands.runOnce(
                        () -> {
                            velocitySamples.clear();
                            voltageSamples.clear();
                        }),

                // Allow modules to orient
                Commands.run(
                                () -> {
                                    drive.runCharacterization(0.0);
                                },
                                drive)
                        .withTimeout(FF_START_DELAY),

                // Start timer
                Commands.runOnce(timer::restart),

                // Accelerate and gather data
                Commands.run(
                                () -> {
                                    double voltage = timer.get() * FF_RAMP_RATE;
                                    drive.runCharacterization(voltage);
                                    velocitySamples.add(drive.getFFCharacterizationVelocity());
                                    voltageSamples.add(voltage);
                                },
                                drive)

                        // When cancelled, calculate and print results
                        .finallyDo(
                                () -> {
                                    int n = velocitySamples.size();
                                    double sumX = 0.0;
                                    double sumY = 0.0;
                                    double sumXY = 0.0;
                                    double sumX2 = 0.0;
                                    for (int i = 0; i < n; i++) {
                                        sumX += velocitySamples.get(i);
                                        sumY += voltageSamples.get(i);
                                        sumXY += velocitySamples.get(i) * voltageSamples.get(i);
                                        sumX2 += velocitySamples.get(i) * velocitySamples.get(i);
                                    }
                                    double kS =
                                            (sumY * sumX2 - sumX * sumXY)
                                                    / (n * sumX2 - sumX * sumX);
                                    double kV =
                                            (n * sumXY - sumX * sumY) / (n * sumX2 - sumX * sumX);

                                    NumberFormat formatter = new DecimalFormat("#0.00000");
                                    System.out.println(
                                            "********** Drive FF Characterization Results **********");
                                    System.out.println("\tkS: " + formatter.format(kS));
                                    System.out.println("\tkV: " + formatter.format(kV));
                                }));
    }

    /** Measures the robot's wheel radius by spinning in a circle. */
    public static Command wheelRadiusCharacterization(Drive drive) {
        SlewRateLimiter limiter = new SlewRateLimiter(WHEEL_RADIUS_RAMP_RATE);
        WheelRadiusCharacterizationState state = new WheelRadiusCharacterizationState();

        return Commands.parallel(
                // Drive control sequence
                Commands.sequence(
                        // Reset acceleration limiter
                        Commands.runOnce(
                                () -> {
                                    limiter.reset(0.0);
                                }),

                        // Turn in place, accelerating up to full speed
                        Commands.run(
                                () -> {
                                    double speed = limiter.calculate(WHEEL_RADIUS_MAX_VELOCITY);
                                    drive.runVelocity(new ChassisSpeeds(0.0, 0.0, speed));
                                },
                                drive)),

                // Measurement sequence
                Commands.sequence(
                        // Wait for modules to fully orient before starting measurement
                        Commands.waitSeconds(1.0),

                        // Record starting measurement
                        Commands.runOnce(
                                () -> {
                                    state.positions =
                                            drive.getWheelRadiusCharacterizationPositions();
                                    state.lastAngle = drive.getRotation();
                                    state.gyroDelta = 0.0;
                                }),

                        // Update gyro delta
                        Commands.run(
                                        () -> {
                                            var rotation = drive.getRotation();
                                            state.gyroDelta +=
                                                    Math.abs(
                                                            rotation.minus(state.lastAngle)
                                                                    .getRadians());
                                            state.lastAngle = rotation;
                                        })

                                // When cancelled, calculate and print results
                                .finallyDo(
                                        () -> {
                                            double[] positions =
                                                    drive.getWheelRadiusCharacterizationPositions();
                                            double wheelDelta = 0.0;
                                            for (int i = 0; i < 4; i++) {
                                                wheelDelta +=
                                                        Math.abs(positions[i] - state.positions[i])
                                                                / 4.0;
                                            }
                                            double wheelRadius =
                                                    (state.gyroDelta * Drive.DRIVE_BASE_RADIUS)
                                                            / wheelDelta;

                                            NumberFormat formatter = new DecimalFormat("#0.000");
                                            System.out.println(
                                                    "********** Wheel Radius Characterization Results **********");
                                            System.out.println(
                                                    "\tWheel Delta: "
                                                            + formatter.format(wheelDelta)
                                                            + " radians");
                                            System.out.println(
                                                    "\tGyro Delta: "
                                                            + formatter.format(state.gyroDelta)
                                                            + " radians");
                                            System.out.println(
                                                    "\tWheel Radius: "
                                                            + formatter.format(wheelRadius)
                                                            + " meters, "
                                                            + formatter.format(
                                                                    Units.metersToInches(
                                                                            wheelRadius))
                                                            + " inches");
                                        })));
    }

    private static class WheelRadiusCharacterizationState {
        double[] positions = new double[4];
        Rotation2d lastAngle = new Rotation2d();
        double gyroDelta = 0.0;
    }

    public static Command driveToAuto(
            RobotContainer r, Supplier<Pose2d> destination, boolean isGather) {
        Command c;
        // NOTE: path complete is set to true in NewPathFinder.init()
        if (isGather && false) {
            c = new NewPathFinder(r, destination, isGather);
        } else {
            c =
                    new NewPathFinder(r, destination, isGather)
                            .andThen(driveToPoint(r, destination, isGather));
        }
        c =
                c.finallyDo(
                        () -> {
                            r.state.pathComplete = true;
                            r.state.inLocalPosePhase = false;
                        });
        c = c.alongWith(new InstantCommand(() -> r.state.onTarget = false));
        c.setName("DriveToAuto");
        return c;
    }

    public static Command driveTo(
            RobotContainer r, Supplier<Pose2d> destination, boolean isGather) {
        Command driveTo = driveToAuto(r, destination, isGather).andThen(joystickDriveFlysky(r));

        if (isGather) {
            // if close to the gather station, dont repath to it
            return new ConditionalCommand(
                    joystickDriveFlysky(r),
                    driveTo,
                    () ->
                            r.drive
                                            .chooseLocalPose()
                                            .getTranslation()
                                            .getDistance(destination.get().getTranslation())
                                    < Units.inchesToMeters(36));
        } else {
            return driveTo;
        }
    }

    public static Command oldDriveTo(
            RobotContainer r, Supplier<Pose2d> destination, boolean isGather) {
        Transform2d finderDelta = new Transform2d(Units.feetToMeters(-1.5), 0, Rotation2d.kZero);
        Supplier<Pose2d> farDestination =
                new Supplier<Pose2d>() {
                    public Pose2d get() {
                        return destination.get().plus(finderDelta);
                    }
                };

        return new ConditionalCommand(
                        new PathfindingCommand(r, farDestination, isGather)
                                .andThen(new PathFollowingCommand(r, destination, isGather)),
                        new PathFollowingCommand(r, destination, isGather),
                        () ->
                                r.drive
                                                .getGlobalPose()
                                                .getTranslation()
                                                .getDistance(destination.get().getTranslation())
                                        > Units.feetToMeters(6))
                .andThen(driveToPoint(r, destination, isGather));
    }

    public static Command leaveReef(RobotContainer r) {
        return new RunCommand(() -> r.drive.runVelocity(new ChassisSpeeds(-2, 0, 0)))
                .raceWith(new WaitCommand(0.5));
    }

    public static Command driveVel(RobotContainer r, ChassisSpeeds speeds) {
        return new RunCommand(() -> r.drive.runVelocity(speeds), r.drive).finallyDo(r.drive::stop);
    }

    public static Command driveFieldVel(RobotContainer r, ChassisSpeeds speeds) {
        return new RunCommand(
                () -> {
                    if (Locations.isBlue()) {
                        r.drive.runVelocity(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        speeds, r.drive.getRotation()));
                    } else {
                        r.drive.runVelocity(
                                ChassisSpeeds.fromFieldRelativeSpeeds(
                                        speeds, r.drive.getRotation().plus(Rotation2d.k180deg)));
                    }
                });
    }

    public static Command zeroDrive(RobotContainer r) {
        if (Constants.currentMode == Mode.SIM) {
            return new InstantCommand(
                    () -> r.drive.setPose(r.drive.driveSimulation.getSimulatedDriveTrainPose()));
        } else {
            return new ConditionalCommand(
                    new InstantCommand(
                            () ->
                                    r.drive.setPose(
                                            new Pose2d(
                                                    r.drive.getGlobalPose().getTranslation(),
                                                    new Rotation2d()))),
                    new InstantCommand(
                            () ->
                                    r.drive.setPose(
                                            new Pose2d(
                                                    r.drive.getGlobalPose().getTranslation(),
                                                    Rotation2d.k180deg))),
                    () -> Locations.isBlue());
        }
    }

    public static Command zeroDrive60(RobotContainer r) {
        return new ConditionalCommand(
                new InstantCommand(
                        () ->
                                r.drive.setPose(
                                        new Pose2d(
                                                r.drive.getGlobalPose().getTranslation(),
                                                new Rotation2d(Units.degreesToRadians(60))))),
                new InstantCommand(
                        () ->
                                r.drive.setPose(
                                        new Pose2d(
                                                r.drive.getGlobalPose().getTranslation(),
                                                new Rotation2d(Units.degreesToRadians(240))))),
                () -> Locations.isBlue());
    }

    public static Command zeroDriven60(RobotContainer r) {
        return new ConditionalCommand(
                new InstantCommand(
                        () ->
                                r.drive.setPose(
                                        new Pose2d(
                                                r.drive.getGlobalPose().getTranslation(),
                                                new Rotation2d(Units.degreesToRadians(-60))))),
                new InstantCommand(
                        () ->
                                r.drive.setPose(
                                        new Pose2d(
                                                r.drive.getGlobalPose().getTranslation(),
                                                new Rotation2d(Units.degreesToRadians(-240))))),
                () -> Locations.isBlue());
    }

    public static Command waitUntilClose(RobotContainer r, Supplier<Pose2d> target, double dist) {
        return new WaitUntilCommand(
                () ->
                        r.drive
                                        .chooseLocalPose()
                                        .getTranslation()
                                        .getDistance(target.get().getTranslation())
                                < Units.inchesToMeters(dist));
    }
}
