package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorIO {
  @AutoLog
  public static class ElevatorIOInputs {
    public boolean elevatorConnected = false;
    public double elevatorPositionInches = 0;
    public double elevatorVelocityInchesPerSec = 0;
    public double elevatorAppliedVolts = 0;
    public double elevatorCurrentAmps = 0;
    public double elevatorTempFahrenheit = 0;
  }

  public default void updateInputs(ElevatorIOInputs inputs) {}

  public default void setElevatorVelocity(double velocity) {}

  public default void setElevatorVolts(double volts) {}

  public default void setElevatorPosition(double motorPosition) {}
}
