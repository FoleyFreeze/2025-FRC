package frc.robot.subsystems.climb;

import org.littletonrobotics.junction.AutoLog;

public interface ClimbIO {

    @AutoLog
    public static class ClimbIOInputs {
        public boolean climbConnected = false;
        public double climbPositionRads = 0;
        public double climbVelocityRadPerSec = 0;
        public double climbAppliedVolts = 0;
        public double climbCurrentAmps = 0;
        public double climbTempFahrenheit = 0;
        public double climbAbsPosition = 0;
    }

    public default void updateInputs(ClimbIOInputs inputs) {}

    public default void setClimbVolts(double volts) {}

    public default void zero() {}
}
