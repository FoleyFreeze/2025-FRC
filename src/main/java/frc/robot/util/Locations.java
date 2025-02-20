package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;

public class Locations {

    static double robotWidth = Units.inchesToMeters(28 + 6);
    static double robotLength = Units.inchesToMeters(30 + 6);
    static Transform2d halfRobot = new Transform2d(robotLength / 2, 0, new Rotation2d());
    static Transform2d halfRobotCoral =
            new Transform2d(
                    robotLength / 2 + Units.inchesToMeters(4),
                    Units.inchesToMeters(5),
                    new Rotation2d());

    public static AprilTagFieldLayout tags =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    public static Pose2d getLeftGatherStation() {
        Pose2d tag;

        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get().equals(Alliance.Red)) {
            tag = tags.getTagPose(1).get().toPose2d().plus(halfRobot);
        } else {
            tag = tags.getTagPose(13).get().toPose2d().plus(halfRobot);
        }
        Pose2d output =
                new Pose2d(tag.getTranslation(), tag.getRotation().plus(Rotation2d.k180deg));
        return output;
    }

    public static Pose2d getRightGatherStation() {
        Pose2d tag;

        if (DriverStation.getAlliance().isPresent()
                && DriverStation.getAlliance().get().equals(Alliance.Red)) {
            tag = tags.getTagPose(2).get().toPose2d().plus(halfRobot);
        } else {
            tag = tags.getTagPose(12).get().toPose2d().plus(halfRobot);
        }
        Pose2d output =
                new Pose2d(tag.getTranslation(), tag.getRotation().plus(Rotation2d.k180deg));
        return output;
    }

    public static Pose2d getTag7() {
        Pose2d tag = tags.getTagPose(7).get().toPose2d().plus(halfRobotCoral);
        Pose2d output =
                new Pose2d(tag.getTranslation(), tag.getRotation().plus(Rotation2d.k180deg));
        return output;
    }
}
