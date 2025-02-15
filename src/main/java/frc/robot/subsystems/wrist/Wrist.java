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

    Angle target = null;
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

        Logger.recordOutput("Wrist/Setpoint", target == null ? 0 : target.in(Radians));

        Logger.recordOutput("Wrist/LocalAngle", cvrtEncToLocal(inputs.wristPositionRad, r.arm.getAngle().in(Radians)));
        Logger.recordOutput("Wrist/MinBound", cvrtLocalToEnc(k.minLocalWristAngleCoral.in(Radians), r.arm.getAngle().in(Radians)));
        Logger.recordOutput("Wrist/MaxBound", cvrtLocalToEnc(k.maxLocalWristAngle.in(Radians), r.arm.getAngle().in(Radians)));
    }

    public double getVoltage() {
        return (inputs.wristAppliedVolts);
    }

    public Command goTo(Supplier<SuperstructureLocation> loc) {
        return new RunCommand(
                        () -> 
                            setAngle(loc.get().wristAngle)
                        ,
                        this)
                .until(() -> atTarget())
                .finallyDo(
                        b -> {
                            if (!b)
                                System.out.format(
                                        "Wrist completed at %.1f with err %.1f\n",
                                        target.in(Degrees),
                                        Units.radiansToDegrees(inputs.wristPositionRad)
                                                - target.in(Degrees));
                        });
    }

    public void setAngle(Angle angle) {
        target = angle;
        double minAngle = cvrtLocalToEnc(k.minLocalWristAngleCoral.in(Radians), r.arm.getAngle().in(Radians));
        double maxAngle = cvrtLocalToEnc(k.maxLocalWristAngle.in(Radians), r.arm.getAngle().in(Radians));

        double newAngleTarget = MathUtil.clamp(angle.in(Radians), minAngle, maxAngle);
        io.setWristPosition(newAngleTarget);
    }

    public Angle getAngleRads() {
        return Radians.of(inputs.wristPositionRad);
    }

    public boolean atTarget() {
        return Math.abs(target.in(Radians) - inputs.wristPositionRad) < k.closeEnough;
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
        io.zero();
    }

    public double cvrtLocalToEnc(double localWristAngle, double armAngle){
        double extraArm = (armAngle + 83) / k.g3;
        double wristEncAngle = localWristAngle - extraArm;
        return wristEncAngle;
    }

    public double cvrtEncToLocal(double wristAngle, double armAngle){
        double extraArm = (armAngle + 83) / k.g3;
        double wristLocalAngle = wristAngle + extraArm;
        return wristLocalAngle;
    }
}
