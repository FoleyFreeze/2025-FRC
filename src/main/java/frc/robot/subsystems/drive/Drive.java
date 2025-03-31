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

package frc.robot.subsystems.drive;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.CANBus;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.PathPlannerLogging;
import edu.wpi.first.hal.FRCNetComm.tInstances;
import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.RobotContainer;
import frc.robot.generated.TunerConstants;
import frc.robot.util.LocalADStarAK;
import frc.robot.util.Locations;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;
import java.util.function.Consumer;
import org.ironmaple.simulation.SimulatedArena;
import org.ironmaple.simulation.drivesims.COTS;
import org.ironmaple.simulation.drivesims.SwerveDriveSimulation;
import org.ironmaple.simulation.drivesims.configs.DriveTrainSimulationConfig;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

public class Drive extends SubsystemBase {
    // TunerConstants doesn't include these constants, so they are declared locally
    static final double ODOMETRY_FREQUENCY =
            new CANBus(TunerConstants.DrivetrainConstants.CANBusName).isNetworkFD() ? 250.0 : 100.0;
    public static final double DRIVE_BASE_RADIUS =
            Math.max(
                    Math.max(
                            Math.hypot(
                                    TunerConstants.FrontLeft.LocationX,
                                    TunerConstants.FrontLeft.LocationY),
                            Math.hypot(
                                    TunerConstants.FrontRight.LocationX,
                                    TunerConstants.FrontRight.LocationY)),
                    Math.max(
                            Math.hypot(
                                    TunerConstants.BackLeft.LocationX,
                                    TunerConstants.BackLeft.LocationY),
                            Math.hypot(
                                    TunerConstants.BackRight.LocationX,
                                    TunerConstants.BackRight.LocationY)));

    // PathPlanner config constants
    private static final double ROBOT_MASS_KG = 74.088;
    private static final double ROBOT_MOI = 6.883;
    private static final double WHEEL_COF = 1.2;
    private static final RobotConfig PP_CONFIG =
            new RobotConfig(
                    ROBOT_MASS_KG,
                    ROBOT_MOI,
                    new ModuleConfig(
                            TunerConstants.FrontLeft.WheelRadius,
                            TunerConstants.kSpeedAt12Volts.in(MetersPerSecond),
                            WHEEL_COF,
                            DCMotor.getKrakenX60Foc(1)
                                    .withReduction(TunerConstants.FrontLeft.DriveMotorGearRatio),
                            TunerConstants.FrontLeft.SlipCurrent,
                            1),
                    getModuleTranslations());

    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Module[] modules = new Module[4]; // FL, FR, BL, BR
    private final SysIdRoutine sysId;
    private final Alert gyroDisconnectedAlert =
            new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);

    private final Alert driveTempAlert = new Alert("Drive Motor Temp > 150", AlertType.kWarning);

    private SwerveDriveKinematics kinematics = new SwerveDriveKinematics(getModuleTranslations());
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
            new SwerveModulePosition[] {
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition(),
                new SwerveModulePosition()
            };
    private SwerveDrivePoseEstimator globalPoseEstimator =
            new SwerveDrivePoseEstimator(
                    kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private SwerveDrivePoseEstimator localPoseEstimator =
            new SwerveDrivePoseEstimator(
                    kinematics, rawGyroRotation, lastModulePositions, new Pose2d());

    private ChassisSpeeds robotVelocity = new ChassisSpeeds();

    private final Consumer<Pose2d> resetSimulationPoseCallBack;

    public SwerveDriveSimulation driveSimulation;

    RobotContainer r;

    public static Drive create(RobotContainer r) {
        Drive drive;
        switch (Constants.currentMode) {
            case REAL:
                drive =
                        new Drive(
                                new GyroIOPigeon2(),
                                // new GyroIO() {},
                                new ModuleIOTalonFX(TunerConstants.FrontLeft),
                                new ModuleIOTalonFX(TunerConstants.FrontRight),
                                new ModuleIOTalonFX(TunerConstants.BackLeft),
                                new ModuleIOTalonFX(TunerConstants.BackRight),
                                (pose) -> {});
                // drive =
                //         new Drive(
                //                 new GyroIO() {},
                //                 new ModuleIO() {},
                //                 new ModuleIO() {},
                //                 new ModuleIO() {},
                //                 new ModuleIO() {},
                //                 (pose) -> {});
                break;

            case SIM:
                // Create and configure a drivetrain simulation configuration
                final DriveTrainSimulationConfig driveTrainSimulationConfig =
                        DriveTrainSimulationConfig.Default()
                                // Specify gyro type (for realistic gyro drifting and error
                                // simulation)
                                .withGyro(COTS.ofPigeon2())
                                // Specify swerve module (for realistic swerve dynamics)
                                .withSwerveModule(
                                        COTS.ofMark4(
                                                DCMotor.getKrakenX60(
                                                        1), // Drive motor is a Kraken X60
                                                DCMotor.getFalcon500(
                                                        1), // Steer motor is a Falcon 500
                                                COTS.WHEELS
                                                        .COLSONS
                                                        .cof, // Use the COF for Colson Wheels
                                                3)) // L3 Gear ratio
                                // Configures the track length and track width (spacing between
                                // swerve modules)
                                .withTrackLengthTrackWidth(Inches.of(26), Inches.of(24))
                                // Configures the bumper size (dimensions of the robot bumper)
                                .withBumperSize(Inches.of(36), Inches.of(34));
                SwerveDriveSimulation sim =
                        new SwerveDriveSimulation(
                                driveTrainSimulationConfig,
                                // Specify starting pose
                                new Pose2d(3, 3, new Rotation2d()));
                // Register the drivetrain simulation to the default simulation world
                SimulatedArena.getInstance().addDriveTrainSimulation(sim);
                // Sim robot, instantiate physics sim IO implementations
                drive =
                        new Drive(
                                new GyroIOSim(sim.getGyroSimulation()),
                                new ModuleIOSim(sim.getModules()[0]),
                                new ModuleIOSim(sim.getModules()[1]),
                                new ModuleIOSim(sim.getModules()[2]),
                                new ModuleIOSim(sim.getModules()[3]),
                                sim::setSimulationWorldPose);

                drive.driveSimulation = sim;

                break;

            default:
                // Replayed robot, disable IO implementations
                drive =
                        new Drive(
                                new GyroIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                new ModuleIO() {},
                                (pose) -> {});
        }

        drive.r = r;
        return drive;
    }

    public Drive(
            GyroIO gyroIO,
            ModuleIO flModuleIO,
            ModuleIO frModuleIO,
            ModuleIO blModuleIO,
            ModuleIO brModuleIO,
            Consumer<Pose2d> resetSimulationPoseCallBack) {
        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0, TunerConstants.FrontLeft);
        modules[1] = new Module(frModuleIO, 1, TunerConstants.FrontRight);
        modules[2] = new Module(blModuleIO, 2, TunerConstants.BackLeft);
        modules[3] = new Module(brModuleIO, 3, TunerConstants.BackRight);
        this.resetSimulationPoseCallBack = resetSimulationPoseCallBack;

        // Usage reporting for swerve template
        HAL.report(
                tResourceType.kResourceType_RobotDrive, tInstances.kRobotDriveSwerve_AdvantageKit);

        // Start odometry thread
        PhoenixOdometryThread.getInstance().start();

        // Configure AutoBuilder for PathPlanner
        AutoBuilder.configure(
                this::chooseLocalPose,
                this::setPose,
                this::getChassisSpeeds,
                this::runVelocityFF,
                new PPHolonomicDriveController(
                        new PIDConstants(5.0, 0.0, 0.0), new PIDConstants(5.0, 0.0, 0.0)),
                PP_CONFIG,
                () -> false,
                // () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
                this);
        Pathfinding.setPathfinder(new LocalADStarAK());
        PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                            "Odometry/Trajectory",
                            activePath.toArray(new Pose2d[activePath.size()]));
                });
        PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                    double xyError =
                            this.getGlobalPose().minus(targetPose).getTranslation().getNorm();
                    Logger.recordOutput("Odometry/TrajectoryError", xyError);
                });

        // Configure SysId
        sysId =
                new SysIdRoutine(
                        new SysIdRoutine.Config(
                                null,
                                null,
                                null,
                                (state) ->
                                        Logger.recordOutput("Drive/SysIdState", state.toString())),
                        new SysIdRoutine.Mechanism(
                                (voltage) -> runCharacterization(voltage.in(Volts)), null, this));
    }

    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
            module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
            for (var module : modules) {
                module.stop();
            }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
            Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
            Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps =
                modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        robotVelocity = new ChassisSpeeds();
        for (int i = 0; i < sampleCount; i++) {
            // Read wheel positions and deltas from each module
            SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
            SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
            for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
                modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
                moduleDeltas[moduleIndex] =
                        new SwerveModulePosition(
                                modulePositions[moduleIndex].distanceMeters
                                        - lastModulePositions[moduleIndex].distanceMeters,
                                modulePositions[moduleIndex].angle);
                lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
            }

            Twist2d twist = kinematics.toTwist2d(moduleDeltas);
            robotVelocity.vxMetersPerSecond += twist.dx;
            robotVelocity.vyMetersPerSecond += twist.dy;
            robotVelocity.omegaRadiansPerSecond += twist.dtheta;

            robotVelocity = ChassisSpeeds.fromRobotRelativeSpeeds(robotVelocity, getRotation());

            // Update gyro angle
            if (gyroInputs.connected) {
                // Use the real gyro angle
                rawGyroRotation = gyroInputs.odometryYawPositions[i];
            } else {
                // Use the angle delta from the kinematics and module deltas
                rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
            }

            // Apply update
            globalPoseEstimator.updateWithTime(
                    sampleTimestamps[i], rawGyroRotation, modulePositions);
            localPoseEstimator.updateWithTime(
                    sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        robotVelocity.vxMetersPerSecond /= 0.02;
        robotVelocity.vyMetersPerSecond /= 0.02;
        robotVelocity.omegaRadiansPerSecond /= 0.02;

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

        double maxTemp = 0;
        for (var module : modules) {
            double temp = module.inputs.driveTempF;
            if (temp > maxTemp) maxTemp = temp;
            temp = module.inputs.turnTempF;
            if (temp > maxTemp) maxTemp = temp;
        }

        // update temperature alert
        driveTempAlert.set(maxTemp > 150);
    }

    /**
     * Runs the drive at the desired velocity.
     *
     * @param speeds Speeds in meters/sec
     */
    public void runVelocity(ChassisSpeeds speeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);

        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    public void runVelocityFF(ChassisSpeeds speeds, DriveFeedforwards accels) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(speeds, 0.02);
        SwerveModuleState[] setpointStates = kinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, TunerConstants.kSpeedAt12Volts);
        // Log unoptimized setpoints and setpoint speeds
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);
        Logger.recordOutput("SwerveChassisSpeeds/Accel", accels.accelerationsMPSSq());

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i], accels.accelerationsMPSSq()[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }

    /** Runs the drive in a straight line with the specified drive output. */
    public void runCharacterization(double output) {
        for (int i = 0; i < 4; i++) {
            modules[i].runCharacterization(output);
        }
    }

    /** Stops the drive. */
    public void stop() {
        runVelocity(new ChassisSpeeds());
    }

    /**
     * Stops the drive and turns the modules to an X arrangement to resist movement. The modules
     * will return to their normal orientations the next time a nonzero velocity is requested.
     */
    public void stopWithX() {
        Rotation2d[] headings = new Rotation2d[4];
        for (int i = 0; i < 4; i++) {
            headings[i] = getModuleTranslations()[i].getAngle();
        }
        kinematics.resetHeadings(headings);
        stop();
    }

    /** Returns a command to run a quasistatic test in the specified direction. */
    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.quasistatic(direction));
    }

    /** Returns a command to run a dynamic test in the specified direction. */
    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return run(() -> runCharacterization(0.0))
                .withTimeout(1.0)
                .andThen(sysId.dynamic(direction));
    }

    /** Returns the module states (turn angles and drive velocities) for all of the modules. */
    @AutoLogOutput(key = "SwerveStates/Measured")
    private SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getState();
        }
        return states;
    }

    /** Returns the module positions (turn angles and drive positions) for all of the modules. */
    private SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] states = new SwerveModulePosition[4];
        for (int i = 0; i < 4; i++) {
            states[i] = modules[i].getPosition();
        }
        return states;
    }

    /** Returns the measured chassis speeds of the robot. */
    @AutoLogOutput(key = "SwerveChassisSpeeds/Measured")
    private ChassisSpeeds getChassisSpeeds() {
        return kinematics.toChassisSpeeds(getModuleStates());
    }

    /** Returns the position of each module in radians. */
    public double[] getWheelRadiusCharacterizationPositions() {
        double[] values = new double[4];
        for (int i = 0; i < 4; i++) {
            values[i] = modules[i].getWheelRadiusCharacterizationPosition();
        }
        return values;
    }

    /** Returns the average velocity of the modules in rotations/sec (Phoenix native units). */
    public double getFFCharacterizationVelocity() {
        double output = 0.0;
        for (int i = 0; i < 4; i++) {
            output += modules[i].getFFCharacterizationVelocity() / 4.0;
        }
        return output;
    }

    /** Returns the current odometry pose. */
    @AutoLogOutput(key = "Odometry/Robot")
    public Pose2d getGlobalPose() {
        return globalPoseEstimator.getEstimatedPosition();
    }

    @AutoLogOutput(key = "Odometry/LocalRobot")
    public Pose2d getLocalPose() {
        return localPoseEstimator.getEstimatedPosition();
    }

    public Pose2d chooseLocalPose() {
        if (r.state.inLocalPosePhase) {
            return getLocalPose();
        } else {
            /*localPoseEstimator.resetPosition(
            rawGyroRotation, getModulePositions(), getGlobalPose());*/
            return getGlobalPose();
        }
    }

    public void resetLocalPose() {
        localPoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), getGlobalPose());
    }

    /** Returns the current odometry rotation. */
    public Rotation2d getRotation() {
        return getGlobalPose().getRotation();
    }

    // field oriented speeds
    @AutoLogOutput(key = "Odometry/RobotVel")
    public ChassisSpeeds getVelocity() {
        return ChassisSpeeds.fromRobotRelativeSpeeds(getChassisSpeeds(), getRotation());
    }

    /** Resets the current odometry pose. */
    public void setPose(Pose2d pose) {
        resetSimulationPoseCallBack.accept(pose);
        globalPoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
        localPoseEstimator.resetPosition(rawGyroRotation, getModulePositions(), pose);
    }

    /** Adds a new timestamped vision measurement. */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs,
            int id) {
        globalPoseEstimator.addVisionMeasurement(
                visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);

        // only update local odo if its the tag we are going to
        if (triggerAndId(id)) {
            localPoseEstimator.addVisionMeasurement(
                    visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        }
    }

    private boolean triggerAndId(int id) {
        if (r.flysky.rightTriggerSWG.getAsBoolean()) {
            if (r.controlBoard.algaeModeT.getAsBoolean()) {
                return true;
            } else {
                // in coral mode
                if (Locations.getTagId(r.controlBoard.selectedReefPos) == id) {
                    return true;
                } else if (r.controlBoard.selectedLevel == 1) {
                    // use all reef tags for level 1
                    if (Locations.isBlue()) {
                        return id <= 22 && id >= 17;
                    } else {
                        return id <= 11 && id >= 6;
                    }
                }
            }
        } else if (r.flysky.leftTriggerSWE.getAsBoolean()) {
            if (r.controlBoard.algaeModeT.getAsBoolean()) {
                if (Locations.getTagId(r.controlBoard.selectedReefPos) == id) {
                    return true;
                }
            } else if (Locations.getCoralStationTag(r) == id) {
                return true;
            }
        }
        return false;
    }

    /** Returns the maximum linear speed in meters per sec. */
    public double getMaxLinearSpeedMetersPerSec() {
        return TunerConstants.kSpeedAt12Volts.in(MetersPerSecond);
    }

    /** Returns the maximum angular speed in radians per sec. */
    public double getMaxAngularSpeedRadPerSec() {
        return getMaxLinearSpeedMetersPerSec() / DRIVE_BASE_RADIUS;
    }

    /** Returns an array of module translations. */
    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(
                    TunerConstants.FrontLeft.LocationX, TunerConstants.FrontLeft.LocationY),
            new Translation2d(
                    TunerConstants.FrontRight.LocationX, TunerConstants.FrontRight.LocationY),
            new Translation2d(TunerConstants.BackLeft.LocationX, TunerConstants.BackLeft.LocationY),
            new Translation2d(
                    TunerConstants.BackRight.LocationX, TunerConstants.BackRight.LocationY)
        };
    }

    public Translation3d getAccelerometer() {
        return new Translation3d(gyroInputs.accelX, gyroInputs.accelY, gyroInputs.accelZ);
    }

    public void setBrakes(boolean on) {
        for (Module m : modules) {
            m.setBrakes(on);
        }
    }
}
