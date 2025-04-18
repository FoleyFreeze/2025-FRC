package frc.robot.util;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.controls.ControlBoard.ReefSticks;
import java.util.ArrayList;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class PathCache {

    // note cache is regenerated when alliance changes
    RobotContainer r;
    Pose2d[] waypointList;

    Pose2d[] gatherList;

    Translation2d reefPose;

    double velocityThreshold = 0.4;

    public PathCache(RobotContainer r) {
        this.r = r;

        Transform2d offset = new Transform2d(Units.inchesToMeters(-18), 0, Rotation2d.k180deg);
        waypointList = new Pose2d[6];
        waypointList[0] = Locations.getAlgaeReefLocation(ReefSticks.A).plus(offset);
        waypointList[1] = Locations.getAlgaeReefLocation(ReefSticks.C).plus(offset);
        waypointList[2] = Locations.getAlgaeReefLocation(ReefSticks.E).plus(offset);
        waypointList[3] = Locations.getAlgaeReefLocation(ReefSticks.G).plus(offset);
        waypointList[4] = Locations.getAlgaeReefLocation(ReefSticks.I).plus(offset);
        waypointList[5] = Locations.getAlgaeReefLocation(ReefSticks.K).plus(offset);

        reefPose = Locations.getReef();

        gatherList = new Pose2d[2];
        gatherList[0] = Locations.getLeftGatherStationCenter();
        gatherList[1] = Locations.getRightGatherStationCenter();
    }

    public List<Pose2d> getPathTo(Pose2d dest, boolean isGather) {
        return getPathTo(dest, isGather, false);
    }

    public List<Pose2d> getPathTo(Pose2d dest, boolean isGather, boolean isClimb) {
        // find the waypoint closest to the start and dest
        Pose2d start = r.drive.chooseLocalPose();
        int closeStart = 0;
        int closeDest = 0;
        double closestDistStart = getDist(waypointList[0], start);
        double closestDistDest = getDist(waypointList[0], dest);
        for (int i = 1; i < waypointList.length; i++) {
            double distStart = getDist(waypointList[i], start);
            double distDest = getDist(waypointList[i], dest);
            if (distStart < closestDistStart) {
                closestDistStart = distStart;
                closeStart = i;
            }
            if (distDest < closestDistDest) {
                closestDistDest = distDest;
                closeDest = i;
            }
        }

        // determine the positive or negative "count" distance between the start waypoint and the
        // dest waypoint
        int count;
        if (Math.abs(closeDest - closeStart) > 3) {
            if (closeDest - closeStart < 0) {
                count = closeDest - closeStart + 6;
            } else {
                count = closeDest - closeStart - 6;
            }
        } else {
            count = closeDest - closeStart;
        }

        // create a start pose with rotation indicating the direction to drive
        Pose2d currentLocation = r.drive.getGlobalPose();
        ChassisSpeeds currentVel = r.drive.getVelocity();

        // if velocity is high, point the path along the direction of velocity

        Translation2d travelVector;
        Rotation2d travelAngle;
        if (Math.hypot(currentVel.vxMetersPerSecond, currentVel.vyMetersPerSecond)
                > velocityThreshold) {
            travelVector =
                    new Translation2d(currentVel.vxMetersPerSecond, currentVel.vyMetersPerSecond);
            travelAngle = travelVector.getAngle();
        } else {
            if (closeToGather(start)) {
                // point along the robot vector
                travelAngle = start.getRotation();
            } else if (closeToReef(start)) {
                // point along the reef to robot vector
                travelVector = start.getTranslation().minus(reefPose);
                // travelAngle = start.getRotation().plus(Rotation2d.k180deg);
                travelAngle = travelVector.getAngle();
            } else if (count != 0) {
                // point the travel vector at the next waypoint
                int inc = count / Math.abs(count);
                travelVector =
                        waypointList[Math.floorMod(closeStart + inc, 6)]
                                .getTranslation()
                                .minus(currentLocation.getTranslation());
                travelAngle = travelVector.getAngle();
            } else {
                // or if there isnt one, at the destination
                travelVector = dest.getTranslation().minus(currentLocation.getTranslation());
                travelAngle = travelVector.getAngle();
            }
        }

        /*
        Translation2d travelVector =
                waypointList[closeStart].getTranslation().minus(currentLocation.getTranslation());
        */

        Pose2d startPose = new Pose2d(currentLocation.getTranslation(), travelAngle);

        // include all points between the closest start and dest waypoints.
        // ignore the closest points, as the convex hex + offset should allow going to dest from the
        // prev point without hitting the reef
        ArrayList<Pose2d> poseList = new ArrayList<>();
        poseList.add(startPose);
        if (count != 0) {
            int inc = count / Math.abs(count);
            for (int delta = inc; Math.abs(delta) < Math.abs(count); delta += inc) {
                // create new poses from waypoints that point to the midpoint rotation between the
                // prev
                // and next pose
                Pose2d thisPoint = waypointList[Math.floorMod(closeStart + delta, 6)];

                Rotation2d dir = thisPoint.getRotation().plus(Rotation2d.kCCW_90deg.times(inc));
                poseList.add(new Pose2d(thisPoint.getTranslation(), dir));
            }
        }

        if (isClimb) {
            Pose2d finalPose;
            Pose2d backPose;
            if (Locations.isBlue()) {
                backPose =
                        new Pose2d(
                                dest.getTranslation()
                                        .plus(new Translation2d(Units.inchesToMeters(-20), 0)),
                                Rotation2d.kZero);
                finalPose = new Pose2d(dest.getTranslation(), Rotation2d.kZero);
            } else {
                backPose =
                        new Pose2d(
                                dest.getTranslation()
                                        .plus(new Translation2d(Units.inchesToMeters(20), 0)),
                                Rotation2d.k180deg);
                finalPose = new Pose2d(dest.getTranslation(), Rotation2d.k180deg);
            }
            // the final straight drive in
            poseList.add(backPose);
            poseList.add(finalPose);

        } else /*if (!isGather || true)*/ {
            // the final straight drive in
            poseList.add(
                    dest.transformBy(
                            new Transform2d(Units.inchesToMeters(-20), 0, Rotation2d.kZero)));

            poseList.add(dest);
        }

        return poseList;
    }

    public double getDist(Pose2d one, Pose2d two) {
        return one.getTranslation().getDistance(two.getTranslation());
    }

    Transform2d extraBackup = new Transform2d(Units.inchesToMeters(12), 0, Rotation2d.kZero);

    public Pose2d closestWaypoint() {
        return r.drive.getGlobalPose().nearest(List.of(waypointList)).plus(extraBackup);
    }

    public boolean closeToGather(Pose2d bot) {
        boolean isClose = false;
        double dist = 20;
        for (Pose2d p : gatherList) {
            double newDist = p.getTranslation().getDistance(bot.getTranslation());
            if (newDist < dist) dist = newDist;
            if (dist < Units.inchesToMeters(30)) isClose = true;
        }

        Logger.recordOutput("State/GatherDist", Units.metersToInches(dist));
        return isClose;
    }

    public boolean closeToReef(Pose2d bot) {
        boolean isClose = false;
        double dist = reefPose.getDistance(bot.getTranslation());
        if (dist < Units.inchesToMeters(70)) isClose = true;

        Logger.recordOutput("State/ReefDist", Units.metersToInches(dist));
        return isClose;
    }
}
