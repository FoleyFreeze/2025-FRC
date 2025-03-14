package frc.robot.subsystems.cvision;

import edu.wpi.first.networktables.BooleanEntry;
import edu.wpi.first.networktables.DoubleEntry;
import edu.wpi.first.networktables.NetworkTableEvent;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.RawSubscriber;
import edu.wpi.first.wpilibj.Timer;
import java.nio.ByteBuffer;
import java.util.EnumSet;

public class CVisionIO_HW implements CVisionIO {

    private RawSubscriber poseMsgCage;
    private ByteBuffer poseDataCage;

    // private RawSubscriber poseMsgTag;
    // private ByteBuffer poseDataTag;

    private BooleanEntry active;
    private BooleanEntry cagesActive;
    // private BooleanEntry tagsActive;

    CVisionCageData cageData = new CVisionCageData();
    // CVisionTagData tagData = new CVisionTagData(0);

    // based on the 2023-FRC project
    public CVisionIO_HW() {
        active = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Active").getEntry(true);
        cagesActive =
                NetworkTableInstance.getDefault()
                        .getBooleanTopic("/Vision/Cage Enable")
                        .getEntry(true);
        // tagsActive = NetworkTableInstance.getDefault().getBooleanTopic("/Vision/Tag
        // Enable").getEntry(true);
        active.set(true);
        cagesActive.set(true);
        // tagsActive.set(true);

        poseMsgCage =
                NetworkTableInstance.getDefault()
                        .getTable("Vision")
                        .getRawTopic("Cage Pose Data Bytes")
                        .subscribe("raw", null);
        NetworkTableInstance.getDefault()
                .addListener(
                        poseMsgCage,
                        EnumSet.of(NetworkTableEvent.Kind.kValueAll),
                        (event) -> {
                            poseDataCage = ByteBuffer.wrap(event.valueData.value.getRaw());
                            byte type = poseDataCage.get(12);
                            byte numTags = poseDataCage.get(13);
                            int seqNum = poseDataCage.getInt(0);
                            float current = (float) Timer.getFPGATimestamp();
                            float timeStamp =
                                    current
                                            - ((current
                                                            - poseDataCage.getFloat(4)
                                                            + poseDataCage.getFloat(8))
                                                    / 2.0f);

                            CVisionCageData vd = null;
                            if (numTags > 0) {
                                vd =
                                        new CVisionCageData(
                                                seqNum,
                                                timeStamp,
                                                poseDataCage.getFloat(18),
                                                (float) Math.toRadians(poseDataCage.getFloat(14)));
                            }
                            if (vd != null) {
                                cageData = vd;
                            }
                        });

        // poseMsgTag = NetworkTableInstance.getDefault().getTable("Vision").getRawTopic("Tag Pose
        // Data Bytes").subscribe("raw", null);
        // NetworkTableInstance.getDefault().addListener(poseMsgTag,
        //     EnumSet.of(NetworkTableEvent.Kind.kValueAll),
        //     event -> {
        //         poseDataTag = ByteBuffer.wrap(event.valueData.value.getRaw());
        //         byte type = poseDataTag.get(12); // type 1 = tag, type 2 = cone, type 3 = cube
        //         byte numTags = poseDataTag.get(13);
        //         if(type == 1 && numTags <= 4){//only supports 4 tags in one frame
        //             CVisionTagData e = new CVisionTagData(numTags);
        //             e.seqNum = poseDataTag.getInt(0);
        //             float current = Logger.getRealTimestamp()/1000000.0f;
        //             e.timestamp = current - ((current -  poseDataTag.getFloat(4) +
        // poseDataTag.getFloat(8)) / 2.0f);
        //             // added tag decision margin (1 float) and error bits (1 byte) to message
        //             // this takes each tag struct from 25 bytes to 30 bytes
        //             // old: for(int i = 0, b = 14; i < numTags; i++, b += 25){
        //             for(int i = 0, b = 14; i < numTags; i++, b += 30) {

        //                 CVisionTag visionData = new CVisionTag(poseDataTag.get(b),
        //                     poseDataTag.get(b+1), poseDataTag.getFloat(b+2),
        //                     poseDataTag.getFloat(b+6), poseDataTag.getFloat(b+10),
        // poseDataTag.getFloat(b+14),
        //                     poseDataTag.getFloat(b+18), poseDataTag.getFloat(b+22),
        // poseDataTag.getFloat(b+26));
        //                 e.tags[i] = visionData;

        //             }
        //             tagData = e;
        //         }

        //     });
    }

    DoubleEntry rioTime =
            NetworkTableInstance.getDefault().getDoubleTopic("/Vision/RIO Time").getEntry(0);

    @Override
    public void updateInputs(CVisionIOInputs inputs) {
        inputs.cageData = cageData;
        // inputs.tagData = tagData;

        double time = Timer.getFPGATimestamp();
        rioTime.set(time);
        inputs.now = time;

        active.set(true);
        cagesActive.set(true);
        // tagsActive.set(true);
    }
}
