package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import java.util.function.Supplier;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SuperstructureLocation;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

    private final ElevatorIO io;
    private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

    private SuperstructureLocation target = null;
    private Timer targetTime = new Timer();
    double targetPosition;
    double targetVelocity;
    State goalState;
    State currentState;
    private TrapezoidProfile trajectory =
            new TrapezoidProfile(new Constraints(40.0, 40.0)); // maxVel, maxAccel

    public static Elevator create() {
        Elevator elevator;
        switch (Constants.currentMode) {
            case REAL:
                elevator = new Elevator(new ElevatorIOHardware(new ElevatorCals()));
                break;

            case SIM:
                elevator = new Elevator(new ElevatorIOSim(new ElevatorCals()));
                break;

            default:
                elevator = new Elevator(new ElevatorIO() {});
                break;
        }

        return elevator;
    }

    public Elevator(ElevatorIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Elevator", inputs);

        if (target != null) {
            // motion profile to target
            State command = trajectory.calculate(0.02, currentState, goalState);
            Logger.recordOutput("Elevator/Target", goalState.position);
            Logger.recordOutput("Elevator/CommandPos", command.position);
            Logger.recordOutput("Elevator/CommandVel", command.velocity);
            currentState = command;
            // PID to change velocity based on position error
            double kp = 2;
            double ks = 1.3;
            double kv = 0.5;
            double err = command.position - inputs.elevatorPositionInches;
            // command.velocity += err * p;
            io.setElevatorVolts(command.velocity * kv + err * kp + ks);
        }
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
        return new InstantCommand(() ->{
            target = loc.get();
            goalState = new State(target.eleHeight.in(Inches), 0);
            currentState =
                    new State(inputs.elevatorPositionInches, inputs.elevatorVelocityInchesPerSec);
            targetTime.restart();
        }, this);
    }

    public void stop() {
        target = null;
        io.setElevatorVolts(0);
    }
}
