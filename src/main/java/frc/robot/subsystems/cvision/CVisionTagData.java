package frc.robot.subsystems.cvision;

import edu.wpi.first.util.struct.StructSerializable;

public class CVisionTagData implements StructSerializable {
    public int seqNum;
    public float timestamp;
    public boolean isProcessed;
    public byte tagCount;
    public CVisionTag[] tags;

    public CVisionTagData(int tagCount) {
        this.tagCount = (byte) tagCount;
        tags = new CVisionTag[tagCount];
    }

    public CVisionTagData(int seqNum, boolean isProcessed, float timestamp, CVisionTag[] tags) {
        this.seqNum = seqNum;
        this.isProcessed = isProcessed;
        this.timestamp = timestamp;
        tagCount = (byte) tags.length;
        this.tags = tags;
    }

    public static final CVisionTagDataStruct struct = new CVisionTagDataStruct();
}
