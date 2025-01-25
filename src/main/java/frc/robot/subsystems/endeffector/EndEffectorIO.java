package frc.robot.subsystems.endeffector;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface EndEffectorIO {
  @AutoLog
  public static class EndEffectorIOInputs {
    public Angle endEffectorPosition = Radians.of(0);
    public AngularVelocity endEffectorVelocity = RadiansPerSecond.of(0);
    public Voltage endEffectorAppliedVolts = Volts.of(0);
    public Current endEffectorCurrent = Amps.of(0);
    public Temperature endEffectorTemp = Fahrenheit.of(0);
    public Angle absEncAngle = Radians.of(0);
  }

  public default void updateInputs(EndEffectorIOInputs inputs) {}

  public default void setEndEffectorVolts(Voltage volts) {}

  public default void setEndEffectorPosition(Angle motorPosition) {}
}
