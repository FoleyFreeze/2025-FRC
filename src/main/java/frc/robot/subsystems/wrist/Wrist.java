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
import frc.robot.commands.SuperstructureLocation;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    SuperstructureLocation target = null;
    WristCals k;

    public static Wrist create() {
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
        return wrist;
    }

    public Wrist(WristIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);

        Logger.recordOutput("Wrist/Setpoint", target == null ? 0 : target.wristAngle.in(Radians));
    }

    public double getVoltage() {
        return (inputs.wristAppliedVolts);
    }

    public Command goTo(Supplier<SuperstructureLocation> loc) {
        return new RunCommand(
                        () -> {
                            target = loc.get();
                            io.setWristPosition(target.wristAngle.in(Radians));
                        },
                        this)
                .until(() -> atTarget(loc))
                .finallyDo(
                        b -> {
                            if (!b)
                                System.out.format(
                                        "Wrist completed at %.1f with err %.1f\n",
                                        loc.get().wristAngle.in(Degrees),
                                        Units.radiansToDegrees(inputs.wristPositionRad)
                                                - loc.get().wristAngle.in(Degrees));
                        });
    }

    public Command goToLimit(Supplier<SuperstructureLocation> loc) {
        return new InstantCommand(
                () -> {
                    target = loc.get();
                    // limit wrist angle to -20 - 100
                    double value =
                            MathUtil.clamp(
                                    target.wristAngle.in(Radians),
                                    Units.degreesToRadians(-15),
                                    Units.degreesToRadians(100));
                    io.setWristPosition(value);
                    System.out.format(
                            "Wrist targeting at %.1f instead of %.1f\n",
                            Units.radiansToDegrees(value), loc.get().wristAngle.in(Degrees));
                },
                this);
    }

    public void setAngle(Angle angle) {
        io.setWristPosition(angle.in(Radians));
    }

    public Angle getAngleRads() {
        return Radians.of(inputs.wristPositionRad);
    }

    public boolean atTarget(Supplier<SuperstructureLocation> loc) {
        double target = loc.get().wristAngle.in(Radians);
        double curr = inputs.wristPositionRad;

        return Math.abs(target - curr) < k.closeEnough;
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
}
