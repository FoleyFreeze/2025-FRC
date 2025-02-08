package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Radians;

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

public class Arm extends SubsystemBase {
    private final ArmIO io;
    private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

    public ArmCals k;

    SuperstructureLocation target = null;

    public static Arm create() {
        Arm arm;
        ArmCals cals = new ArmCals();
        switch (Constants.currentMode) {
            case REAL:
                arm = new Arm(new ArmIOHardware(cals));
                break;

            case SIM:
                arm = new Arm(new ArmIOSim(cals));
                break;

            default:
                arm = new Arm(new ArmIO() {});
                break;
        }

        arm.k = cals;
        return arm;
    }

    public Arm(ArmIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Arm", inputs);

        Logger.recordOutput("Arm/Setpoint", target == null ? 0 : target.armAngle.in(Radians));
    }

    public Angle getAngleRads() {
        return Radians.of(inputs.armPositionRad);
    }

    public void setAngle(Angle angle) {
        io.setArmPosition(angle.in(Radians));
    }

    public Command goTo(Supplier<SuperstructureLocation> loc) {
        return new RunCommand(
                        () -> {
                            target = loc.get();
                            io.setArmPosition(target.armAngle.in(Radians));
                        },
                        this)
                .until(() -> atTarget(loc))
                .finallyDo(
                        b -> {
                            if (!b)
                                System.out.format(
                                        "Arm completed at %.1f with err %.1f\n",
                                        loc.get().armAngle.in(Degrees),
                                        Units.radiansToDegrees(inputs.armPositionRad)
                                                - loc.get().armAngle.in(Degrees));
                        });
    }

    public boolean atTarget(Supplier<SuperstructureLocation> loc) {
        double target = loc.get().armAngle.in(Radians);
        double curr = inputs.armPositionRad;

        return Math.abs(target - curr) < k.closeEnough;
    }

    public Command stop() {
        return new InstantCommand(
                () -> {
                    io.setArmVolts(0);
                    target = null;
                },
                this);
    }
}
