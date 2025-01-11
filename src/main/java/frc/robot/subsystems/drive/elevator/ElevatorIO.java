package frc.robot.subsystems.drive.elevator;

import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;

import org.littletonrobotics.junction.AutoLog;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;
import frc.robot.subsystems.drive.ModuleIO.ModuleIOInputs;

public interface ElevatorIO {
    @AutoLog
    public static class ElevatorIOInputs {
        public boolean elevatorConnected = false;
        public Angle elevatorPosition = Radians.of(0);
        public AngularVelocity elevatorVelocity = RadiansPerSecond.of(0);
        public Voltage elevatorAppliedVolts = 0.0;
        public Current elevatorCurrent = 0.0;
        public Temperature elevatorTemp = 0.0;
    }

    
  public default void updateInputs(ModuleIOInputs inputs) {}
    
  public default void setElevatorVelocity(AngularVelocity velocity) {}

  public default void setElevatorVolts(Voltage volts) {}

  public default void setElevatorPosition(Angle motorPosition) {}
}
