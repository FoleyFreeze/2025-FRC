package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.*;

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

        public double absEncAngleRaw = 0;
        public double absEncAngleRel = 0;
    }

    public default void updateInputs(WristIOInputs inputs) {}

    public default void setWristVolts(double volts) {}

    public default void setWristPosition(double rads) {}

    public default void zero() {}

    public default void superZero() {}
}
