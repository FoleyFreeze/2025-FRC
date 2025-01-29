package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;
import org.littletonrobotics.junction.AutoLog;

public interface WristIO {
  @AutoLog
  public static class WristIOInputs {
    public boolean wristConnected = false;
    public double wristPositionRad = 0;
    public double wristVelocityRadPerSec = 0;
    public double wristAppliedVolts = 0;
    public double wristCurrent = 0;
    public double wristTempF = 0;
    public double absEncAngleRad = 0;
  }

  public default void updateInputs(WristIOInputs inputs) {}

  public default void setWristVolts(Voltage volts) {}

  public default void setWristPosition(Angle motorPosition) {}
}
