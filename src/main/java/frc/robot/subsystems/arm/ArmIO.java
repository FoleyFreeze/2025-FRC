package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface ArmIO {
  @AutoLog
  public static class ArmIOInputs {
    public boolean armConnected = false;
    public double armPositionRad = 0;
    public double armVelocityRadPerSec = 0;
    public double armAppliedVolts = 0;
    public double armCurrent = 0;
    public double armTempF = 0;
    public double absEncAngleRad = 0;
  }

  public default void updateInputs(ArmIOInputs inputs) {}

  public default void setArmVolts(Voltage volts) {}

  public default void setArmVelocity(double velocity) {}

  public default void setArmPosition(Angle motorPosition) {}
}
