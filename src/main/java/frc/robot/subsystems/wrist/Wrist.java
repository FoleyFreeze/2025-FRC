package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SuperstructureLocation;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    SuperstructureLocation target = null;
    WristCals k;
    RobotContainer r;

    public static Wrist create(RobotContainer r) {
        Wrist wrist;
        WristCals cals = new WristCals();
        switch (Constants.currentMode) {
            case REAL:
                wrist = new Wrist(new WristIOHardware(cals));
                break;

            case SIM:
                wrist = new Wrist(new WristIOSim(cals));
                break;

            default:
                wrist = new Wrist(new WristIO() {});
                break;
        }
        wrist.k = cals;
        wrist.r = r;
        return wrist;
    }

    public Wrist(WristIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        Logger.recordOutput("Wrist/Setpoint", target == null ? 0 : target.wristAngle.in(Radians));

        Logger.recordOutput(
                "Wrist/LocalAngle",
                cvrtEncToLocal(inputs.wristPositionRad, r.arm.getAngle().in(Radians)));
        Logger.recordOutput(
                "Wrist/MinBound",
                cvrtLocalToEnc(
                        k.minLocalWristAngleCoral.in(Radians), r.arm.getAngle().in(Radians)));
        Logger.recordOutput(
                "Wrist/MaxBound",
                cvrtLocalToEnc(
                        k.maxLocalWristAngleCoral.in(Radians), r.arm.getAngle().in(Radians)));
    }

    public double getVoltage() {
        return (inputs.wristAppliedVolts);
    }

    public Command goTo(Supplier<SuperstructureLocation> loc) {
        return new RunCommand(() -> setAngle(loc.get()), this)
                .until(() -> atTarget(loc))
                .finallyDo(
                        b -> {
                            if (!b)
                                System.out.format(
                                        "Wrist completed at %.1f with err %.1f\n",
                                        target.wristAngle.in(Degrees),
                                        Units.radiansToDegrees(inputs.wristPositionRad)
                                                - target.wristAngle.in(Degrees));
                        });
    }

    public Command goToReally(Supplier<SuperstructureLocation> loc) {
        return new RunCommand(
                        () -> {
                            io.setWristPosition(loc.get().wristAngle.in(Radians));
                            this.target = loc.get();
                        },
                        this)
                .until(() -> atTarget(loc))
                .finallyDo(
                        b -> {
                            if (!b)
                                System.out.format(
                                        "Wrist completed at %.1f with err %.1f\n",
                                        target.wristAngle.in(Degrees),
                                        Units.radiansToDegrees(inputs.wristPositionRad)
                                                - target.wristAngle.in(Degrees));
                        });
    }

    public void setAngle(SuperstructureLocation target) {
        this.target = target;
        double angleTarget = target.wristAngle.in(Radians);

        // if the funnel is in the way, go to -90 degrees local
        if (target.armAngle.in(Degrees) < -30) {
            // we are going toward the funnel
            angleTarget =
                    cvrtLocalToEnc(Units.degreesToRadians(-110), r.arm.getAngle().in(Radians));
        } else if (r.arm.getAngle().in(Degrees) < -35) {
            // we are going away from the funnel, but we are not there yet
            angleTarget = cvrtLocalToEnc(Units.degreesToRadians(-90), r.arm.getAngle().in(Radians));
        }
        Logger.recordOutput("Wrist/ArmAngle", r.arm.getAngle().in(Radians));
        double minAngle, maxAngle;
        if (r.flysky.topLeftSWA.getAsBoolean()) { // Coral Algae sw
            minAngle =
                    cvrtLocalToEnc(k.minLocalWristAngle.in(Radians), r.arm.getAngle().in(Radians));
            maxAngle =
                    cvrtLocalToEnc(k.maxLocalWristAngle.in(Radians), r.arm.getAngle().in(Radians));
        } else {
            minAngle =
                    cvrtLocalToEnc(
                            k.minLocalWristAngleCoral.in(Radians), r.arm.getAngle().in(Radians));
            maxAngle =
                    cvrtLocalToEnc(
                            k.maxLocalWristAngleCoral.in(Radians), r.arm.getAngle().in(Radians));
        }

        double newAngleTarget = MathUtil.clamp(angleTarget, minAngle, maxAngle);

        Logger.recordOutput("Wrist/SetpointBounded", newAngleTarget);
        io.setWristPosition(newAngleTarget);
    }

    public Angle getAngleRads() {
        return Radians.of(inputs.wristPositionRad);
    }

    public boolean atTarget(Supplier<SuperstructureLocation> loc) {
        return Math.abs(loc.get().wristAngle.in(Radians) - inputs.wristPositionRad) < k.closeEnough;
    }

    public Command stop() {
        return new InstantCommand(
                () -> {
                    io.setWristVolts(0);
                    target = null;
                },
                this);
    }

    public void zero() {
        io.superZero();
    }

    public double cvrtLocalToEnc(double localWristAngle, double armAngle) {
        double extraArm = (armAngle + Units.degreesToRadians(83)) / k.g3;
        double wristEncAngle = localWristAngle + extraArm;
        return wristEncAngle;
    }

    public double cvrtEncToLocal(double wristAngle, double armAngle) {
        double extraArm = (armAngle + Units.degreesToRadians(83)) / k.g3;
        double wristLocalAngle = wristAngle - extraArm;
        return wristLocalAngle;
    }
}
