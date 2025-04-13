package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.ConstraintsZone;
import com.pathplanner.lib.path.EventMarker;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.RotationTarget;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Locations;
import java.util.Collections;
import java.util.List;
import org.littletonrobotics.junction.Logger;

public class BargePathFinder extends Command {

    static int repaths = 0;

    RobotContainer r;
    private Command c;

    // vel, accel, rotvel, rotaccel
    PathConstraints coarseConstraints = new PathConstraints(3, 3.5, 4, 4);
    PathConstraints midConstraints = new PathConstraints(2.2, 3, 4, 4);
    PathConstraints finalConstraints = new PathConstraints(1.2, 1, 3, 2);

    public BargePathFinder(RobotContainer r) {
        this.r = r;
    }

    @Override
    public void initialize() {
        // this is the second to last point, need to drive the 20deg offset 24in from here
        Pose2d targetPose = Locations.getDriveToNetPose(r.drive.getGlobalPose());

        List<Pose2d> poseList = r.pathCache.getPathTo(targetPose, true);

        // use our own offset to generate the last 2, make that 3 waypoints
        Translation2d finalVector = Locations.getDriveToNetOffset();
        Pose2d penultimatePose = new Pose2d(targetPose.getTranslation(), finalVector.getAngle());
        Pose2d finalPose =
                new Pose2d(
                        targetPose.getTranslation().plus(Locations.getDriveToNetOffset()),
                        finalVector.getAngle());

        double extraMidBuffer = 0;
        if (Locations.yValInThresh(targetPose.getY()) && false) {
            // add third waypoint
            Pose2d theThird = Locations.getNetWaypoint();

            // modify target pose to shift a bit more left
            targetPose =
                    new Pose2d(
                            targetPose.getX(),
                            // this is a delta of ~24in inverted for red
                            targetPose.getY() + finalVector.getY() * 4,
                            targetPose.getRotation());

            // recalc final poses
            penultimatePose = new Pose2d(targetPose.getTranslation(), finalVector.getAngle());
            finalPose =
                    new Pose2d(
                            targetPose.getTranslation().plus(Locations.getDriveToNetOffset()),
                            finalVector.getAngle());

            poseList.set(poseList.size() - 2, theThird);
            poseList.set(poseList.size() - 1, penultimatePose);
            poseList.add(finalPose);

            extraMidBuffer = 0.75;
        } else {
            // just drive
            poseList.set(poseList.size() - 2, penultimatePose);
            poseList.set(poseList.size() - 1, finalPose);
        }

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(poseList);

        // always use the back of the robot
        Rotation2d angleToTag =
                targetPose
                        .getTranslation()
                        .minus(Locations.getNetTag().getTranslation())
                        .getAngle();
        Pose2d flipPose;
        flipPose = new Pose2d(finalPose.getTranslation(), angleToTag);

        double startMovingThingsPosition = 0;

        ConstraintsZone cz =
                new ConstraintsZone(waypoints.size() - 2, waypoints.size() - 1, finalConstraints);
        ConstraintsZone cz2 =
                new ConstraintsZone(
                        waypoints.size() - 2.5 - extraMidBuffer,
                        waypoints.size() - 2,
                        midConstraints);

        RotationTarget rt = new RotationTarget(waypoints.size() - 2, flipPose.getRotation());
        EventMarker em =
                new EventMarker(
                        "InSlowDrivePhase", startMovingThingsPosition, waypoints.size() - 1);
        int minWaypoints = waypoints.size() - 3;
        if (minWaypoints < 0) minWaypoints = 0;
        EventMarker em2 = new EventMarker("InLocalPosePhase", minWaypoints, -1);
        EventMarker em3 =
                new EventMarker("ShootForNet", waypoints.size() - 2, waypoints.size() - 1);

        // ChassisSpeeds botVel = r.drive.getVelocity();
        // double speed = Math.hypot(botVel.vxMetersPerSecond, botVel.vyMetersPerSecond);
        // Rotation2d angle = new Rotation2d(botVel.vxMetersPerSecond, botVel.vyMetersPerSecond);

        PathPlannerPath path =
                new PathPlannerPath(
                        waypoints,
                        List.of(rt),
                        Collections.emptyList(),
                        List.of(cz2, cz),
                        List.of(em, em2, em3),
                        coarseConstraints,
                        null,
                        new GoalEndState(finalConstraints.maxVelocityMPS(), flipPose.getRotation()),
                        false);

        r.state.pathComplete = false;
        c = AutoBuilder.followPath(path);
        c.initialize();
    }

    @Override
    public void execute() {
        c.execute();

        // reinit if we are failing to follow the path
        if (r.drive.lastPathError > Units.inchesToMeters(24)) {
            initialize();
            Logger.recordOutput("Odometry/BargeRepaths", ++repaths);
        }
    }

    @Override
    public boolean isFinished() {
        return c.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        // if we are in the process of interrupting ourselves
        if (interrupted && r.shootForNet.getAsBoolean() && r.state.algaeNetStage2) {
            // save this command to r so it can be resumed in part 2
            r.activePath = c;
        } else {
            c.end(interrupted);
            r.drive.runVelocity(new ChassisSpeeds());

            // make sure nothing is still stored in r
            r.activePath = null;
        }
    }

    public Command getPathFollower() {
        return c;
    }
}
