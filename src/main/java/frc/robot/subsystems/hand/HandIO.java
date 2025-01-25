package frc.robot.subsystems.hand;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface HandIO {
  @AutoLog
  public static class HandIOInputs {
    public Voltage handAppliedVolts = Volts.of(0);
    public Current handCurrent = Amps.of(0);
    public Temperature handTemp = Fahrenheit.of(0);
  }

  public default void updateInputs(HandIOInputs inputs) {}

  public default void setHandVolts(Voltage volts) {}
}
