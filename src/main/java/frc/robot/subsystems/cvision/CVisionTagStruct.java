package frc.robot.subsystems.cvision;

import edu.wpi.first.util.struct.Struct;
import java.nio.ByteBuffer;

public class CVisionTagStruct implements Struct<CVisionTag> {
    @Override
    public Class<CVisionTag> getTypeClass() {
        return CVisionTag.class;
    }

    @Override
    public String getTypeString() {
        return "struct:visionTag";
    }

    @Override
    public String getTypeName() {
        return "struct:CVisionTagStruct";
    }

    @Override
    public int getSize() {
        return kSizeInt8 * 2 + kSizeFloat * 7;
    }

    @Override
    public String getSchema() {
        return "int8 tagId;int8 eBits;float decisionMargin;float transX;float transY;float transZ;float rotX;float rotY;float rotZ";
    }

    @Override
    public Struct<?>[] getNested() {
        return new Struct<?>[] {};
    }

    @Override
    public CVisionTag unpack(ByteBuffer bb) {
        byte tagId = bb.get();
        byte eBits = bb.get();
        float decisionMargin = bb.getFloat();
        float transX = bb.getFloat();
        float transY = bb.getFloat();
        float transZ = bb.getFloat();
        float rotX = bb.getFloat();
        float rotY = bb.getFloat();
        float rotZ = bb.getFloat();
        return new CVisionTag(
                tagId, eBits, decisionMargin, rotX, rotY, rotZ, transX, transY, transZ);
    }

    @Override
    public void pack(ByteBuffer bb, CVisionTag value) {
        bb.put(value.tagId);
        bb.put(value.eBits);
        bb.putFloat(value.decisionMargin);
        bb.putFloat(value.transX);
        bb.putFloat(value.transY);
        bb.putFloat(value.transZ);
        bb.putFloat(value.rotX);
        bb.putFloat(value.rotY);
        bb.putFloat(value.rotZ);
    }
}
