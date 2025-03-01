package frc.robot.util;

import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import frc.robot.RobotContainer;
import frc.robot.subsystems.controls.ControlBoard.ReefSticks;
import java.util.ArrayList;
import java.util.List;

public class PathCache {

    // note cache is regenerated when alliance changes
    RobotContainer r;
    Pose2d[] waypointList;

    public PathCache(RobotContainer r) {
        this.r = r;

        Transform2d offset = new Transform2d(Units.inchesToMeters(18), 0, Rotation2d.k180deg);
        waypointList = new Pose2d[6];
        waypointList[0] = Locations.getAlgaeReefLocation(ReefSticks.A).plus(offset);
        waypointList[1] = Locations.getAlgaeReefLocation(ReefSticks.C).plus(offset);
        waypointList[2] = Locations.getAlgaeReefLocation(ReefSticks.E).plus(offset);
        waypointList[3] = Locations.getAlgaeReefLocation(ReefSticks.G).plus(offset);
        waypointList[4] = Locations.getAlgaeReefLocation(ReefSticks.I).plus(offset);
        waypointList[5] = Locations.getAlgaeReefLocation(ReefSticks.K).plus(offset);
    }

    public List<Waypoint> getPathTo(Pose2d dest) {
        // find the waypoint closest to the start and dest
        Pose2d start = r.drive.getPose();
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
        if (Math.abs(closeStart - closeDest) > 3) {
            if (closeStart - closeDest < 0) {
                count = closeStart - closeDest + 3;
            } else {
                count = closeStart - closeDest - 3;
            }
        } else {
            count = closeStart - closeDest;
        }

        // create a start pose with rotation indicating the direction to drive
        Pose2d currentLocation = r.drive.getPose();
        Translation2d travelVector =
                waypointList[closeStart].getTranslation().minus(currentLocation.getTranslation());
        Pose2d startPose = new Pose2d(currentLocation.getTranslation(), travelVector.getAngle());

        // include all points between the closest start and dest waypoints.
        // ignore the closest points, as the convex hex + offset should allow going to dest from the
        // prev point without hitting the reef
        ArrayList<Pose2d> poseList = new ArrayList<>();
        poseList.add(startPose);
        Translation2d prevPoint = startPose.getTranslation();
        int inc = count / Math.abs(count);
        for (int delta = inc; Math.abs(delta) < Math.abs(count); delta += inc) {
            // create new poses from waypoints that point to the midpoint rotation between the prev
            // and next pose
            Pose2d thisPoint = waypointList[(closeStart + delta) % 6];
            Pose2d nextPoint = waypointList[(closeStart + delta + inc) % 6];
            Translation2d toPoint = thisPoint.getTranslation().minus(prevPoint);
            Translation2d fromPoint = nextPoint.getTranslation().minus(thisPoint.getTranslation());
            Rotation2d avgRot = toPoint.getAngle().plus(fromPoint.getAngle()).times(0.5);
            poseList.add(new Pose2d(thisPoint.getTranslation(), avgRot));
            prevPoint = thisPoint.getTranslation();
        }

        // fix the last pose to point at the dest
        Pose2d thisPoint = poseList.get(poseList.size() - 1);
        Translation2d toPoint = thisPoint.getTranslation().minus(prevPoint);
        Translation2d fromPoint = dest.getTranslation().minus(thisPoint.getTranslation());
        Rotation2d avgRot = toPoint.getAngle().plus(fromPoint.getAngle()).times(0.5);
        poseList.add(poseList.size() - 1, new Pose2d(thisPoint.getTranslation(), avgRot));

        poseList.add(dest);

        return PathPlannerPath.waypointsFromPoses(poseList);
    }

    public double getDist(Pose2d one, Pose2d two) {
        return one.getTranslation().getDistance(two.getTranslation());
    }
}
