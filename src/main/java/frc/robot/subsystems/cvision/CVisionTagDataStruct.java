package frc.robot.subsystems.cvision;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class CVisionTagDataStruct implements Struct<CVisionTagData> {

    static final CVisionTag emptyTag = new CVisionTag();

    @Override
    public Class<CVisionTagData> getTypeClass() {
        return CVisionTagData.class;
    }

    @Override
    public String getTypeString() {
        return "struct:visionTagData";
    }

    @Override
    public String getTypeName() {
        return "struct:visionTagDataStruct";
    }

    @Override
    public int getSize() {
        return kSizeInt32 + kSizeBool + kSizeInt8 + kSizeFloat + CVisionTag.struct.getSize() * 4;
    }

    @Override
    public String getSchema() {
        return "int8 seqNum;float timestamp;bool isProcessed;int8 tagCount;VisionTag tag1;VisionTag tag2;VisionTag tag3;VisionTag tag4";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {CVisionTag.struct};
    }

    @Override
    public CVisionTagData unpack(ByteBuffer bb) {
        int seqNum = bb.getInt();
        float timestamp = bb.getFloat();
        boolean isProcessed = bb.get() > 0;
        byte tagCount = bb.get();
        CVisionTag[] tags = new CVisionTag[tagCount];
        for (int i = 0; i < 4; i++) {
            if (i < tagCount) {
                tags[i] = CVisionTag.struct.unpack(bb);
            } else {
                CVisionTag.struct.unpack(bb);
            }
        }
        return new CVisionTagData(seqNum, isProcessed, timestamp, tags);
    }

    @Override
    public void pack(ByteBuffer bb, CVisionTagData value) {
        bb.putInt(value.seqNum);
        bb.putFloat(value.timestamp);
        bb.put(value.isProcessed ? (byte) 1 : 0);
        bb.put(value.tagCount);
        for (int i = 0; i < 4; i++) {
            if (i < value.tagCount) {
                CVisionTag.struct.pack(bb, value.tags[i]);
            } else {
                CVisionTag.struct.pack(bb, emptyTag);
            }
        }
    }
}
