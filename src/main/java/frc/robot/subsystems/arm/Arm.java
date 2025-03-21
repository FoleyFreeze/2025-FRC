package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Radians;

import com.revrobotics.spark.ClosedLoopSlot;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

    private final Alert armDisconnectedAlert = new Alert("Arm Disconnected", AlertType.kError);

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
        SmartDashboard.putNumber("ArmAbs", inputs.absEncAngleRaw);

        armDisconnectedAlert.set(!inputs.armConnected);
    }

    public Angle getAngle() {
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
        /*.finallyDo(
        b -> {
            if (!b)
                System.out.format(
                        "Arm completed at %.1f with err %.1f\n",
                        loc.get().armAngle.in(Degrees),
                        Units.radiansToDegrees(inputs.armPositionRad)
                                - loc.get().armAngle.in(Degrees));
        })*/ ;
    }

    public boolean atTarget(Supplier<SuperstructureLocation> loc, boolean extraTol) {
        double target = loc.get().armAngle.in(Radians);
        double curr = inputs.armPositionRad;

        double tol = extraTol ? k.closeEnough * 3 : k.closeEnough;
        return Math.abs(target - curr) < tol;
    }

    public boolean atTarget(Supplier<SuperstructureLocation> loc) {
        return atTarget(loc, false);
    }

    public Command stop() {
        return new InstantCommand(
                () -> {
                    io.setArmVolts(0);
                    target = null;
                },
                this);
    }

    public void zero() {
        io.superZero();
    }

    public Command setVoltage(double volts) {
        Command c = new InstantCommand(() -> io.setArmVolts(volts), this);
        return c;
    }

    public void setPIDSlot(ClosedLoopSlot slot) {
        io.setPIDSlot(slot);
        System.out.println("Arm PID set to slot: " + slot);
    }

    public void setBrake(boolean on) {
        io.setBrake(on);
    }
}
