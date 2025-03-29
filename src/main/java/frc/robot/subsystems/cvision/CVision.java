package frc.robot.subsystems.cvision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.CircularBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
// import frc.robot.auton.Locations;
// import frc.robot.cals.VisionCals;
import org.littletonrobotics.junction.Logger;

public class CVision extends SubsystemBase {
    RobotContainer r;
    public CVisionCals k;
    CVisionIO io;
    CVisionIOInputsAutoLogged inputs = new CVisionIOInputsAutoLogged();

    Translation2d lastCageLocation;

    int badIdErr = 0;
    int badHeightErr = 0;
    int badXErr = 0;

    public class TimestampedPose2d {
        Pose2d pose;
        double time;
    }

    CircularBuffer<TimestampedPose2d> robotPoseBuffer;

    public CVision(RobotContainer r, CVisionCals k) {
        this.r = r;
        this.k = k;

        if (Robot.isReal() && !k.disable) {
            io = new CVisionIO_HW();
        } else {
            io = new CVisionIO() {};
        }

        robotPoseBuffer = new CircularBuffer<>(k.bufferSize);
    }

    public static CVision create(RobotContainer r) {
        return new CVision(r, new CVisionCals());
    }

    private void updatePoseBuffer() {
        TimestampedPose2d now = new TimestampedPose2d();
        now.pose = r.drive.getGlobalPose();
        now.time = inputs.now;
        robotPoseBuffer.addFirst(now);
    }

    private Pose2d posePicker(double time) {
        TimestampedPose2d prev = robotPoseBuffer.getFirst();
        for (int i = 0; i < robotPoseBuffer.size(); i++) {
            TimestampedPose2d next = robotPoseBuffer.get(i);
            double delta = next.time - time;
            if (delta < 0) {
                double t = ((time - next.time) / (prev.time - next.time));
                return next.pose.interpolate(prev.pose, t);
            }
        }
        // if the time is before everything in the buffer return the oldest thing
        return robotPoseBuffer.getLast().pose;
    }

    public Translation2d calcCageLocation() {
        // data from camera has the field in (x,z) plane and rotation around the y axis
        // we turn that into a pose2d representation
        // distance provided is only the z component (on axis)
        // we must calculate the true distance (hypotenuse)
        // also the camera angle is +cw which is backwards compared to ours

        double dist = Units.inchesToMeters(inputs.cageData.distance);
        double angle = -inputs.cageData.angle;
        double radius = dist / Math.cos(angle);
        Translation2d camRelCageLoc = new Translation2d(radius, new Rotation2d(angle));
        Logger.recordOutput("Vision/camRelCageLoc", camRelCageLoc);

        // camera relative -> bot relative -> field relative
        Translation2d roboRelCageLocation =
                camRelCageLoc
                        .rotateBy(k.camLocation.getRotation())
                        .plus(k.camLocation.getTranslation());
        Logger.recordOutput("Vision/roboRelCageLocation", roboRelCageLocation);
        Pose2d robotPose = posePicker(inputs.cageData.timeStamp);
        Logger.recordOutput("Vision/robotPose", robotPose);
        Translation2d fieldRelCageLocation =
                roboRelCageLocation
                        .rotateBy(robotPose.getRotation())
                        .plus(robotPose.getTranslation());

        // done in periodic
        // Logger.recordOutput("Vision/fieldRelCageLocation", fieldRelCageLocation);

        // mark as processed
        inputs.cageData.isProcessed = true;

        return fieldRelCageLocation;
    }

    public Translation2d getCachedCageLocation() {
        return lastCageLocation;
    }

    public boolean hasCageImage() {
        return inputs.now - inputs.cageData.timeStamp < k.maxCageAge;
    }

    public boolean hasNewCageImage() {
        return !inputs.cageData.isProcessed && hasCageImage();
    }

    // public boolean hasTagImage(){
    //     return inputs.now - inputs.tagData.timestamp < k.maxTagAge;
    // }

    // public boolean hasNewTagImage(){
    //     return !inputs.tagData.isProcessed && hasTagImage();
    // }

    static final Translation2d zeroT2D = new Translation2d();

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Vision", inputs);

        updatePoseBuffer();

        if (hasNewCageImage()) {
            lastCageLocation = calcCageLocation();
            Logger.recordOutput("Vision/fieldRelCageLocation", lastCageLocation);
        } else {
            // Logger.recordOutput("Vision/fieldRelCageLocation", zeroT2D);
        }

        /*
        if(hasNewTagImage()){
            for(int i=0;i<inputs.tagData.tagCount;i++){
                CVisionTag tag = inputs.tagData.tags[i];

                //create pose3d
                Pose3d rawTagPose = new Pose3d(new Translation3d(tag.transX, tag.transY, tag.transZ), new Rotation3d(tag.rotX, tag.rotY, tag.rotZ));
                Logger.recordOutput("Vision/RawTagPose", rawTagPose);
                Logger.recordOutput("Vision/RawTag/RotX", Math.toDegrees(rawTagPose.getRotation().getX()));
                Logger.recordOutput("Vision/RawTag/RotY", Math.toDegrees(rawTagPose.getRotation().getY()));
                Logger.recordOutput("Vision/RawTag/RotZ", Math.toDegrees(rawTagPose.getRotation().getZ()));

                Pose3d tempPose = rawTagPose.rotateBy(k.tagCamLocation.getRotation());
                Pose3d rawRobotPose = new Pose3d(tempPose.getTranslation().plus(k.tagCamLocation.getTranslation()), tempPose.getRotation());
                Logger.recordOutput("Vision/RawRobotTagPose", rawRobotPose);
                Logger.recordOutput("Vision/RawRobotTag/RotX", Math.toDegrees(rawRobotPose.getRotation().getX()));
                Logger.recordOutput("Vision/RawRobotTag/RotY", Math.toDegrees(rawRobotPose.getRotation().getY()));
                Logger.recordOutput("Vision/RawRobotTag/RotZ", Math.toDegrees(rawRobotPose.getRotation().getZ()));


                Pose2d robotPose = posePicker(inputs.tagData.timestamp);
                Pose3d botFieldPose = new Pose3d(new Translation3d(robotPose.getX(), robotPose.getY(), 0), new Rotation3d(0, 0, robotPose.getRotation().getRadians()));
                tempPose = rawRobotPose = rawRobotPose.rotateBy(botFieldPose.getRotation());
                Pose3d tagFieldPose = new Pose3d(tempPose.getTranslation().plus(botFieldPose.getTranslation()), tempPose.getRotation());
                Logger.recordOutput("Vision/FieldTagPose", tagFieldPose);
                Logger.recordOutput("Vision/FieldTag/RotX", Math.toDegrees(tagFieldPose.getRotation().getX()));
                Logger.recordOutput("Vision/FieldTag/RotY", Math.toDegrees(tagFieldPose.getRotation().getY()));
                Logger.recordOutput("Vision/FieldTag/RotZ", Math.toDegrees(tagFieldPose.getRotation().getZ()));

                //first check height vs reality to reject incorrect heights
                Optional<Pose3d> fieldTagPose = Locations.tagLayout.getTagPose(tag.tagId);
                if(!fieldTagPose.isPresent()) {
                    System.out.println("Tag: " + tag.tagId + " does not exist");
                    badIdErr++;
                    Logger.recordOutput("Vision/ErrorBadId", badIdErr);
                    break;
                }

                double targetHeight = fieldTagPose.get().getZ();
                if(Math.abs(targetHeight - rawRobotPose.getZ()) > Units.inchesToMeters(4)){
                    System.out.println("Tag height doesn't match the field id:" + tag.tagId + " height: " + Units.metersToInches(rawRobotPose.getZ()));
                    badHeightErr++;
                    Logger.recordOutput("Vision/ErrorBadHeight", badHeightErr);
                    break;
                }

                if(tag.transX < 0){
                    System.out.println("Tag is located behind the camera?!? id: " + tag.tagId + " x: " + Units.metersToInches(tag.transX));
                    badXErr++;
                    Logger.recordOutput("Vision/ErrorBadDist", badXErr);
                    break;
                }





            }

            inputs.tagData.isProcessed = true;

        }
        */
    }
}
