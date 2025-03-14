package frc.robot.subsystems.cvision;

import org.littletonrobotics.junction.AutoLog;

public interface CVisionIO {

    @AutoLog
    public static class CVisionIOInputs {
        CVisionCageData cageData = new CVisionCageData();
        // CVisionTagData tagData = new CVisionTagData(0);
        double now;
    }

    public default void updateInputs(CVisionIOInputs inputs) {}
}
