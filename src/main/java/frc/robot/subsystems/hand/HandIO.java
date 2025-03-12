package frc.robot.subsystems.hand;

import static edu.wpi.first.units.Units.*;

import org.littletonrobotics.junction.AutoLog;

public interface HandIO {
    @AutoLog
    public static class HandIOInputs {
        public boolean handConnected = false;
        public double handAppliedVolts = 0;
        public double handCurrent = 0;
        public double handTempF = 0;
    }

    public default void updateInputs(HandIOInputs inputs) {}

    public default void setHandVolts(double volts) {} 

    public default void changeCurrentLimit(double newLimit) {}

    public default void zero() {}
}
