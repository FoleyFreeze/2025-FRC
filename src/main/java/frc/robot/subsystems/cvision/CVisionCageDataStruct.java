package frc.robot.subsystems.cvision;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class CVisionCageDataStruct implements Struct<CVisionCageData> {

    @Override
    public Class<CVisionCageData> getTypeClass() {
        return CVisionCageData.class;
    }

    @Override
    public String getTypeString() {
        return "struct:CVisionCageData";
    }

    @Override
    public String getTypeName() {
        return "struct:CVisionCageDataStruct";
    }

    @Override
    public int getSize() {
        return kSizeInt32 + kSizeFloat * 3 + 1;
    }

    @Override
    public String getSchema() {
        return "int seqNum;float timeStamp;float distance;float angle;boolean isProcessed";
    }

    @Override
    public CVisionCageData unpack(ByteBuffer bb) {
        CVisionCageData v = new CVisionCageData();
        v.seqNum = bb.getInt();
        v.timeStamp = bb.getFloat();
        v.distance = bb.getFloat();
        v.angle = bb.getFloat();
        v.isProcessed = bb.get() > 0;
        return v;
    }

    @Override
    public void pack(ByteBuffer bb, CVisionCageData value) {
        bb.putInt(value.seqNum);
        bb.putFloat(value.timeStamp);
        bb.putFloat(value.distance);
        bb.putFloat(value.angle);
        bb.put(value.isProcessed ? (byte) 1 : (byte) 0);
    }
}
