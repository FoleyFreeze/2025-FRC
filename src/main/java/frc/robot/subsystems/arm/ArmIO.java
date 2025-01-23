package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean armConnected = false;
    public Angle armPosition = Radians.of(0);
    public AngularVelocity armVelocity = RadiansPerSecond.of(0);
    public Voltage armAppliedVolts = Volts.of(0);
    public Current armCurrent = Amps.of(0);
    public Temperature armTemp = Fahrenheit.of(0);
    public Angle absEncAngle = Radians.of(0);
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmVolts(Voltage volts) {}

  public default void setArmPosition(Angle motorPosition) {}
}
