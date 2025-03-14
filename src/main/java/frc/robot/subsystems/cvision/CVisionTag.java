package frc.robot.subsystems.cvision;

public class CVisionTag {
    public byte tagId;
    public byte eBits;
    public float decisionMargin;
    // public Pose3d pose; use floats for smaller log sizes
    public float transX, transY, transZ;
    public float rotX, rotY, rotZ;

    public CVisionTag() {}

    public CVisionTag(
            byte tagId,
            byte eBits,
            float decisionMargin,
            float rotX,
            float rotY,
            float rotZ,
            float transX,
            float transY,
            float transZ) {
        this.tagId = tagId;
        this.eBits = eBits;
        this.decisionMargin = decisionMargin;
        /*this.transX = (float) Units.inchesToMeters(transZ);
        this.transY = (float) Units.inchesToMeters(-transX);
        this.transZ = (float) Units.inchesToMeters(transY);
        this.rotX = (float) Units.degreesToRadians(rotZ);
        this.rotY = (float) Units.degreesToRadians(-rotX);
        this.rotZ = (float) Units.degreesToRadians(rotY);*/
        this.transX = transZ;
        this.transY = -transX;
        this.transZ = -transY;
        this.rotX = rotZ;
        this.rotY = -rotX;
        this.rotZ = -rotY;
    }

    public static final CVisionTagStruct struct = new CVisionTagStruct();
}
