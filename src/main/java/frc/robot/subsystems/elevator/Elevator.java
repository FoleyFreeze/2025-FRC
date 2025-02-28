package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SuperstructureLocation;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    public ElevatorCals k;

    SuperstructureLocation target = null;

    public static Elevator create() {
        Elevator elevator;
        ElevatorCals cals = new ElevatorCals();
        switch (Constants.currentMode) {
            case REAL:
                elevator = new Elevator(new ElevatorIOHardware(cals));
                break;

            case SIM:
                elevator = new Elevator(new ElevatorIOSim(cals));
                break;

            default:
                elevator = new Elevator(new ElevatorIO() {});
                break;
        }

        elevator.k = cals;
        return elevator;
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        Logger.recordOutput("Elevator/Setpoint", target == null ? 0 : target.eleHeight.in(Inches));
    }

    public Distance getHeight() {
        return Inches.of(inputs.elevatorPositionInches);
    }

    public void setHeight(Distance height) {
        // convert from distance to motor rotations
        double position = (height.in(Inches));
        io.setElevatorPosition(position);
    }

    public Command goTo(Supplier<SuperstructureLocation> loc) {
        return new RunCommand(
                        () -> {
                            target = loc.get();
                            io.setElevatorPosition(loc.get().eleHeight.in(Inches));
                        },
                        this)
                .until(() -> atTarget(loc))
        /*.finallyDo(
        b -> {
            if (!b)
                System.out.format(
                        "Elevator completed at %.1f with err %.1f\n",
                        loc.get().eleHeight.in(Inches),
                        inputs.elevatorPositionInches
                                - loc.get().eleHeight.in(Inches));
        })*/ ;
    }

    public Command stop() {
        return new InstantCommand(
                () -> {
                    io.setElevatorVolts(0);
                    target = null;
                },
                this);
    }

    public boolean atTarget(Supplier<SuperstructureLocation> loc) {
        double target = loc.get().eleHeight.in(Inches);
        double curr = inputs.elevatorPositionInches;

        return Math.abs(target - curr) < k.closeEnough;
    }

    public void zero() {
        io.zero();
    }
}
