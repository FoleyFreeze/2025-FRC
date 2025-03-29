package frc.robot.util;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.RobotContainer;
import frc.robot.subsystems.controls.ControlBoard;

public class Locations {

    public static double robotWidth = Units.inchesToMeters(28 + 6);
    public static double robotLength = Units.inchesToMeters(30 + 6);
    public static Transform2d halfRobot = new Transform2d(robotLength / 2.0, 0, new Rotation2d());
    public static Transform2d halfRobotProc =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(6 + 6 /*12 */), 0, new Rotation2d());

    public static Transform2d halfRobotNet =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(1.25 + 1), 0, new Rotation2d());

    // TODO: remove extra before comp
    static double extraGatherX = 8; // 12
    static double extraGatherY = 20 - 4;
    public static Transform2d halfRobotGatherLeftFar =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(2.5 + extraGatherX),
                    Units.inchesToMeters(extraGatherY),
                    Rotation2d.kZero);
    public static Transform2d halfRobotGatherLeftClose =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(2.5 + extraGatherX),
                    Units.inchesToMeters(-extraGatherY),
                    Rotation2d.kZero);
    public static Transform2d halfRobotGatherRightFar =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(2.5 + extraGatherX),
                    Units.inchesToMeters(-extraGatherY),
                    Rotation2d.kZero);
    public static Transform2d halfRobotGatherRightClose =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(2.5 + extraGatherX),
                    Units.inchesToMeters(extraGatherY),
                    Rotation2d.kZero);

    public static Transform2d halfRobotCoralRight =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(3.25),
                    Units.inchesToMeters(5.5),
                    Rotation2d.fromDegrees(1 /*3.5*/));
    public static Transform2d halfRobotCoralLeft =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(3.5),
                    Units.inchesToMeters(-8),
                    Rotation2d.kZero);
    public static Transform2d halfRobotAlgae =
            new Transform2d(
                    robotLength / 2.0 + Units.inchesToMeters(2.5),
                    Units.inchesToMeters(-2.5),
                    Rotation2d.kZero);

    public static Transform2d halfRobotLevel1Left =
            new Transform2d(
                    0,
                    Units.inchesToMeters(-48),
                    Rotation2d.fromDegrees(
                            -82.37)); // not intentionally spefifc, just a gag (please laugh)

    public static Transform2d halfRobotLevel1Right =
            new Transform2d(
                    0,
                    Units.inchesToMeters(48),
                    Rotation2d.fromDegrees(
                            82.37)); // not intentionally spefifc, just a gag (please laugh);

    // TODO: make code that uses this
    // public static Transform2d halfRobotCoralLevel1 =
    //         new Transform2d(
    //                 robotLength / 2.0 + Units.inchesToMeters(7.5),
    //                 Units.inchesToMeters(0),
    //                 new Rotation2d());

    public static AprilTagFieldLayout tags =
            AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded);

    static Pose2d[] blueStarts = {
        new Pose2d(6, 3, Rotation2d.k180deg),
        new Pose2d(6, 3, Rotation2d.k180deg),
        new Pose2d(6, 3, Rotation2d.k180deg)
    };

    static Pose2d[] redStarts = {
        new Pose2d(12, 3, Rotation2d.kZero),
        new Pose2d(12, 3, Rotation2d.kZero),
        new Pose2d(12, 3, Rotation2d.kZero)
    };

    public static Pose2d getStartLoc(int idx) {
        if (isBlue()) {
            if (idx >= blueStarts.length) return null;
            return blueStarts[idx];
        } else {
            if (idx >= redStarts.length) return null;
            return redStarts[idx];
        }
    }

    public static double getNetX() {
        if (isBlue()) {
            return tags.getTagPose(14).get().toPose2d().plus(halfRobotNet).getX();
        } else {
            return tags.getTagPose(5).get().toPose2d().plus(halfRobotNet).getX();
        }
    }

    public static Pose2d getNetPose(Pose2d robotPose) {
        if (isBlue()) {
            double minY = 4.8;
            double maxY = 7.5;
            double y = Math.max(minY, Math.min(robotPose.getY(), maxY));
            return new Pose2d(getNetX(), y, Rotation2d.kZero);
        } else {
            double minY = 0.5;
            double maxY = 3.2;
            double y = Math.max(minY, Math.min(robotPose.getY(), maxY));
            return new Pose2d(getNetX(), y, Rotation2d.k180deg);
        }
    }

    public static Translation2d getReef() {
        Pose2d front, back;
        if (isBlue()) {
            front = tags.getTagPose(18).get().toPose2d();
            back = tags.getTagPose(21).get().toPose2d();
        } else {
            front = tags.getTagPose(7).get().toPose2d();
            back = tags.getTagPose(10).get().toPose2d();
        }

        return front.getTranslation().plus(back.getTranslation()).times(0.5);
    }

    public static Pose2d getLevel1ReefLocation(ControlBoard.ReefSticks position) {
        switch (position) {
            case B:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotLevel1Right));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotLevel1Right));
                }
            case A:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotLevel1Left));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotLevel1Left));
                }
            case D:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotLevel1Right));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotLevel1Right));
                }
            case C:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotLevel1Left));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotLevel1Left));
                }
            case F:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotLevel1Right));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotLevel1Right));
                }
            case E:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotLevel1Left));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotLevel1Left));
                }
            case H:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotLevel1Right));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotLevel1Right));
                }
            case G:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotLevel1Left));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotLevel1Left));
                }
            case J:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotLevel1Right));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotLevel1Right));
                }
            case I:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotLevel1Left));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotLevel1Left));
                }
            case L:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotLevel1Right));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotLevel1Right));
                }
            case K:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotLevel1Left));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotLevel1Left));
                }
            default:
                return null;
        }
    }

    // everyone hates this
    public static Pose2d getReefLocation(ControlBoard.ReefSticks position) {
        switch (position) {
            case B:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralRight));
                }
            case A:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case D:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotCoralRight));
                }
            case C:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case F:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotCoralRight));
                }
            case E:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case H:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotCoralRight));
                }
            case G:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case J:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotCoralRight));
                }
            case I:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotCoralLeft));
                }
            case L:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotCoralRight));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotCoralRight));
                }
            case K:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotCoralLeft));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotCoralLeft));
                }
            default:
                return null;
        }
    }

    public static Pose2d getAlgaeReefLocation(ControlBoard.ReefSticks position) {
        switch (position) {
            case A:
            case B:
            default:
                if (isBlue()) {
                    return invert(tags.getTagPose(18).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(7).get().toPose2d().plus(halfRobotAlgae));
                }
            case C:
            case D:
                if (isBlue()) {
                    return invert(tags.getTagPose(17).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(8).get().toPose2d().plus(halfRobotAlgae));
                }
            case E:
            case F:
                if (isBlue()) {
                    return invert(tags.getTagPose(22).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(9).get().toPose2d().plus(halfRobotAlgae));
                }
            case G:
            case H:
                if (isBlue()) {
                    return invert(tags.getTagPose(21).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(10).get().toPose2d().plus(halfRobotAlgae));
                }
            case I:
            case J:
                if (isBlue()) {
                    return invert(tags.getTagPose(20).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(11).get().toPose2d().plus(halfRobotAlgae));
                }
            case K:
            case L:
                if (isBlue()) {
                    return invert(tags.getTagPose(19).get().toPose2d().plus(halfRobotAlgae));
                } else {
                    return invert(tags.getTagPose(6).get().toPose2d().plus(halfRobotAlgae));
                }
        }
    }

    public static int getTagId(ControlBoard.ReefSticks position) {
        switch (position) {
            case A:
            case B:
            default:
                if (isBlue()) {
                    return 18;
                } else {
                    return 7;
                }
            case C:
            case D:
                if (isBlue()) {
                    return 17;
                } else {
                    return 8;
                }
            case E:
            case F:
                if (isBlue()) {
                    return 22;
                } else {
                    return 9;
                }
            case G:
            case H:
                if (isBlue()) {
                    return 21;
                } else {
                    return 10;
                }
            case I:
            case J:
                if (isBlue()) {
                    return 20;
                } else {
                    return 11;
                }
            case K:
            case L:
                if (isBlue()) {
                    return 19;
                } else {
                    return 6;
                }
        }
    }

    public static Pose2d getLeftGatherStationFar() {
        if (!isBlue()) {
            return invert(tags.getTagPose(1).get().toPose2d().plus(halfRobotGatherLeftFar));
        } else {
            return invert(tags.getTagPose(13).get().toPose2d().plus(halfRobotGatherLeftFar));
        }
    }

    public static Pose2d getRightGatherStationFar() {
        if (!isBlue()) {
            return invert(tags.getTagPose(2).get().toPose2d().plus(halfRobotGatherRightFar));
        } else {
            return invert(tags.getTagPose(12).get().toPose2d().plus(halfRobotGatherRightFar));
        }
    }

    public static Pose2d getLeftGatherStationClose() {
        if (!isBlue()) {
            return invert(tags.getTagPose(1).get().toPose2d().plus(halfRobotGatherLeftClose));
        } else {
            return invert(tags.getTagPose(13).get().toPose2d().plus(halfRobotGatherLeftClose));
        }
    }

    public static Pose2d getRightGatherStationClose() {
        if (!isBlue()) {
            return invert(tags.getTagPose(2).get().toPose2d().plus(halfRobotGatherRightClose));
        } else {
            return invert(tags.getTagPose(12).get().toPose2d().plus(halfRobotGatherRightClose));
        }
    }

    public static Pose2d getTag7() {
        Pose2d tag = tags.getTagPose(7).get().toPose2d().plus(halfRobotCoralLeft);
        return invert(tag);
    }

    public static boolean isBlue() {
        return DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Blue;
    }

    public static Pose2d invert(Pose2d in) {
        return new Pose2d(in.getTranslation(), in.getRotation().plus(Rotation2d.k180deg));
    }

    public static Pose2d getProcLoc() {
        if (isBlue()) {
            return invert(tags.getTagPose(16).get().toPose2d().plus(halfRobotProc));
        } else {
            return invert(tags.getTagPose(3).get().toPose2d().plus(halfRobotProc));
        }
    }

    public static int getCoralStationTag(RobotContainer r) {
        switch (r.controlBoard.selectedStation) {
            case LEFT:
                if (isBlue()) {
                    return 13;
                } else {
                    return 1;
                }
            case RIGHT:
                if (isBlue()) {
                    return 12;
                } else {
                    return 2;
                }
            case CLOSEST:
            default:
                return r.controlBoard.lastGatherStationTag;
        }
    }
}
