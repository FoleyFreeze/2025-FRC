package frc.robot.util;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Locations {

    static double robotWidth = 28+6;
    static double robotLength = 30+6;
    static Transform2d halfRobot = new Transform2d(robotLength/2, 0, new Rotation2d());
    static Transform2d halfRobotCoral = new Transform2d(robotLength/2 + 4, 0, new Rotation2d());
    
    public static AprilTagFieldLayout tags = AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static Pose2d getLeftGatherStation(){
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
            return tags.getTagPose(1).get().toPose2d().plus(halfRobot);
        } else {
            return tags.getTagPose(13).get().toPose2d().plus(halfRobot);
        }
    }

    public static Pose2d getRightGatherStation(){
        if(DriverStation.getAlliance().isPresent() && DriverStation.getAlliance().get().equals(Alliance.Red)){
            return tags.getTagPose(2).get().toPose2d().plus(halfRobot);
        } else {
            return tags.getTagPose(12).get().toPose2d().plus(halfRobot);
        }
    }

    public static Pose2d getTag8(){
        return tags.getTagPose(8).get().toPose2d().plus(halfRobotCoral);
    }

    

}
