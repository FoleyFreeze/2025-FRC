package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean WristConnected = false;
    public Angle WristPosition = Radians.of(0);
    public AngularVelocity WristVelocity = RadiansPerSecond.of(0);
    public Voltage WristAppliedVolts = Volts.of(0);
    public Current WristCurrent = Amps.of(0);
    public Temperature WristTemp = Fahrenheit.of(0);
    public Angle absEncAngle = Radians.of(0);
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setWristVolts(Voltage volts) {}

  public default void setWristPosition(Angle motorPosition) {}
}
