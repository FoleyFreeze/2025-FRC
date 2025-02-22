package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.subsystems.controls.ControlBoard;

public class Locations {

    static double robotWidth = Units.inchesToMeters(28 + 6);
    static double robotLength = Units.inchesToMeters(30 + 6);
    static Transform2d halfRobot = new Transform2d(robotLength / 2, 0, new Rotation2d());
    static Transform2d halfRobotCoralLeft =
            new Transform2d(
                    robotLength / 2 + Units.inchesToMeters(4),
                    Units.inchesToMeters(5),
                    new Rotation2d());
    static Transform2d halfRobotCoralRight =
            new Transform2d(
                    robotLength / 2 + Units.inchesToMeters(4),
                    Units.inchesToMeters(-5),
                    new Rotation2d());

    public static AprilTagFieldLayout tags =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    // everyone hates this
    public static Pose2d getReefLocation(ControlBoard.ReefSticks position) {
        switch (position) {
            case A:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case B:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralRight));
                }
            case C:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case D:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotCoralRight));
                }
            case E:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case F:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotCoralRight));
                }
            case G:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case H:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotCoralRight));
                }
            case I:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case J:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotCoralRight));
                }
            case K:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case L:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotCoralRight));
                }
            default:
                return null;
        }
    }

    public static Pose2d getLeftGatherStation() {
        if (!isBlue()) {
            return invert(tags.getTagPose(1).get().toPose2d().plus(halfRobot));
        } else {
            return invert(tags.getTagPose(13).get().toPose2d().plus(halfRobot));
        }
    }

    public static Pose2d getRightGatherStation() {
        if (!isBlue()) {
            return invert(tags.getTagPose(2).get().toPose2d().plus(halfRobot));
        } else {
            return invert(tags.getTagPose(12).get().toPose2d().plus(halfRobot));
        }
    }

    public static Pose2d getTag7() {
        Pose2d tag = tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralRight);
        return invert(tag);
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    public static Pose2d invert(Pose2d in) {
        return new Pose2d(in.getTranslation(), in.getRotation().plus(Rotation2d.k180deg));
    }
}
