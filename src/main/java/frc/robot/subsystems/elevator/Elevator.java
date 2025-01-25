package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SuperstructureLocation;

public class Elevator extends SubsystemBase {
  private static final Distance radiusElevDrum = Inches.of(1.25);

  private final ElevatorIO io;
  private final ElevatorIOInputsAutoLogged inputs = new ElevatorIOInputsAutoLogged();

  private SuperstructureLocation target = null;
  private Timer targetTime = new Timer();
  private TrapezoidProfile trajectory = new TrapezoidProfile(new Constraints(1.0, 1.0));//maxVel, maxAccel

  public Elevator(ElevatorIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);

    if(target != null){
      //TODO: fix units lib stuff so this can work
      //motion profile to target
      double pos = 0;
      double vel = 0;
      State currentState = new State(pos, vel);
      State goalState = new State(target.eleHeight.in(Meters), 0);
      State command = trajectory.calculate(0.02, currentState, goalState);
      //PID to change velocity based on position error
      double p = 5;
      double err = command.position - pos;
      command.velocity += err * p;
      //io.setElevatorVelocity(command.velocity);
    }
  }

  public Distance getHeight() {
    return radiusElevDrum.times(inputs.elevatorPosition.in(Radians));
  }

  public void setHeight(Distance height) {
    //convert from distance to motor rotations
    Angle position = Radians.of(height.magnitude() / radiusElevDrum.magnitude());
    io.setElevatorPosition(position);
  }

  public void goTo(SuperstructureLocation loc){
    target = loc;
    targetTime.restart();
  }

  public void stop(){
    target = null;
    io.setElevatorVolts(Volts.of(0));
  }

  
}
