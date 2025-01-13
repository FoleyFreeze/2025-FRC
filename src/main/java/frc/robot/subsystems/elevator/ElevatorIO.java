package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean elevatorConnected = false;
    public Angle elevatorPosition = Radians.of(0);
    public AngularVelocity elevatorVelocity = RadiansPerSecond.of(0);
    public Voltage elevatorAppliedVolts = Volts.of(0);
    public Current elevatorCurrent = Amps.of(0);
    public Temperature elevatorTemp = Fahrenheit.of(0);
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setElevatorVelocity(AngularVelocity velocity) {}

  public default void setElevatorVolts(Voltage volts) {}

  public default void setElevatorPosition(Angle motorPosition) {}
}
