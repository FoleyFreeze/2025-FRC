package frc.robot.subsystems.cvision;

import edu.wpi.first.util.struct.StructSerializable;

public class CVisionCageData implements StructSerializable {
    public int seqNum;
    public float timeStamp;
    public float distance;
    public float angle;
    public boolean isProcessed;

    public static final CVisionCageDataStruct struct = new CVisionCageDataStruct();

    public CVisionCageData(int seqNum, float timeStamp, float distance, float angle) {
        this.seqNum = seqNum;
        this.timeStamp = timeStamp;
        this.distance = distance;
        this.angle = angle;
        isProcessed = false;
    }

    public CVisionCageData() {}
}
