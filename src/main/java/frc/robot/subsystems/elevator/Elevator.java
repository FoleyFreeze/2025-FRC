package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SuperstructureLocation;
import org.littletonrobotics.junction.Logger;

public class Elevator extends SubsystemBase {

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private SuperstructureLocation target = null;
  private Timer targetTime = new Timer();
  private TrapezoidProfile trajectory =
      new TrapezoidProfile(new Constraints(10.0, 10.0)); // maxVel, maxAccel

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Elevator", inputs);

    if (target != null) {
      // motion profile to target
      State currentState =
          new State(inputs.elevatorPositionInches, inputs.elevatorVelocityInchesPerSec);
      State goalState = new State(target.eleHeight.in(Inches), 0);
      State command = trajectory.calculate(0.02, currentState, goalState);
      Logger.recordOutput("Elevator/Target", goalState.position);
      Logger.recordOutput("Elevator/CommandPos", command.position);
      Logger.recordOutput("Elevator/CommandVel", command.velocity);
      // PID to change velocity based on position error
      double kp = 1;
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

  public void goTo(SuperstructureLocation loc) {
    target = loc;
    targetTime.restart();
  }

  public void stop() {
    target = null;
    io.setElevatorVolts(0);
  }
}
