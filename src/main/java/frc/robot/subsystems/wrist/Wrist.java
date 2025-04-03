package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final Alert wristDisconnectedAlert = new Alert("Wrist Disconnected", AlertType.kError);

    SuperstructureLocation target = null;
    WristCals k;
    RobotContainer r;

    double level1Jog = 0;
    double level2_3Jog = 0;
    double level4Jog = 0;
    double jogAmount = Math.toRadians(1);

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

        SmartDashboard.putNumber("WristAbs", inputs.absEncAngleRaw);

        wristDisconnectedAlert.set(!inputs.wristConnected);
    }

    public double getVoltage() {
        return (inputs.wristAppliedVolts);
    }

    public Command goTo(Supplier<SuperstructureLocation> loc) {
        return new RunCommand(() -> setAngle(loc.get()), this).until(() -> atTarget(loc))
        /*.finallyDo(
        b -> {
            if (!b)
                System.out.format(
                        "Wrist completed at %.1f with err %.1f\n",
                        target.wristAngle.in(Degrees),
                        Units.radiansToDegrees(inputs.wristPositionRad)
                                - target.wristAngle.in(Degrees));
        })*/ ;
    }

    public Command goToReally(Supplier<SuperstructureLocation> loc) {
        return new RunCommand(
                        () -> {
                            setAngle(loc.get());
                        },
                        this)
                .until(() -> atTarget(loc))
        /*.finallyDo(
        b -> {
            if (!b)
                System.out.format(
                        "Wrist completed at %.1f with err %.1f\n",
                        target.wristAngle.in(Degrees),
                        Units.radiansToDegrees(inputs.wristPositionRad)
                                - target.wristAngle.in(Degrees));
        })*/ ;
    }

    public void setAngle(SuperstructureLocation target) {
        this.target = target;
        double angleTarget = target.wristAngle.in(Radians);

        double jog;
        switch (target) {
            case LEVEL1:
                jog = level1Jog;
                break;
            case LEVEL2:
            case LEVEL3:
                jog = level2_3Jog;
                break;
            case LEVEL4:
                jog = level4Jog;
                break;
            default:
                jog = 0;
        }

        double newAngleTarget = angleTarget + jog;

        Logger.recordOutput("Wrist/SetpointBounded", newAngleTarget);
        io.setWristPosition(newAngleTarget);
    }

    public Angle getAngleRads() {
        return Radians.of(inputs.wristPositionRad);
    }

    public boolean atTarget(Supplier<SuperstructureLocation> loc, boolean extraTol) {
        // for determining position treat the wrist as always there
        if (extraTol) return true;
        double tol = k.closeEnough;
        return Math.abs(loc.get().wristAngle.in(Radians) - inputs.wristPositionRad) < tol;
    }

    public boolean atTarget(Supplier<SuperstructureLocation> loc) {
        return atTarget(loc, false);
    }

    public Command setVoltage(double volts) {
        return new InstantCommand(
                () -> {
                    io.setWristVolts(volts);
                    target = null;
                },
                this);
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

    public void rezeroAbsEnc() {
        io.rezeroAbsEnc();
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

    public void setBrake(boolean on) {
        io.setBrake(on);
    }

    public void jogUp() {
        switch (r.controlBoard.selectedLevel) {
            case 1:
                level1Jog += jogAmount;
                break;
            case 2:
            case 3:
                level2_3Jog += jogAmount;
                break;
            case 4:
                level4Jog += jogAmount;
                break;
            default:
        }

        Logger.recordOutput("Wrist/Jog1", level1Jog);
        Logger.recordOutput("Wrist/Jog2_3", level2_3Jog);
        Logger.recordOutput("Wrist/Jog4", level4Jog);
    }

    public void jogDown() {
        switch (r.controlBoard.selectedLevel) {
            case 1:
                level1Jog -= jogAmount;
                break;
            case 2:
            case 3:
                level2_3Jog -= jogAmount;
                break;
            case 4:
                level4Jog -= jogAmount;
                break;
            default:
        }
        Logger.recordOutput("Wrist/Jog1", level1Jog);
        Logger.recordOutput("Wrist/Jog2_3", level2_3Jog);
        Logger.recordOutput("Wrist/Jog4", level4Jog);
    }
}
