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
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Locations;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;

public class NewPathFinder extends Command {

    static int repaths = 0;

    RobotContainer r;
    boolean isGather;
    boolean isClimb;

    private Supplier<Pose2d> poseSupplier;
    private Command c;

    // vel, accel, rotvel, rotaccel
    PathConstraints coralGlobalConstraints = new PathConstraints(3.5, 4.4, 6, 4);
    // PathConstraints coralGlobalConstraints = new PathConstraints(1, 1, 6, 4);
    PathConstraints algaeGlobalConstraints = new PathConstraints(3, 3.5, 4, 4);
    PathConstraints finalConstraints = new PathConstraints(1.25, 1.5, 3, 2);
    PathConstraints finalConstraintsAlgae = new PathConstraints(1.5, 1.5, 3, 2);
    PathConstraints finalGatherConstraints = new PathConstraints(2, 3, 6, 4);

    public NewPathFinder(
            RobotContainer r, Supplier<Pose2d> poseSupplier, boolean isGather, boolean isClimb) {
        this.poseSupplier = poseSupplier;
        this.r = r;
        this.isGather = isGather;
        this.isClimb = isClimb;
    }

    public NewPathFinder(RobotContainer r, Supplier<Pose2d> poseSupplier, boolean isGather) {
        this.poseSupplier = poseSupplier;
        this.r = r;
        this.isGather = isGather;
        this.isClimb = false;
    }

    @Override
    public void initialize() {
        Pose2d targetPose = poseSupplier.get();

        List<Waypoint> waypoints =
                PathPlannerPath.waypointsFromPoses(
                        r.pathCache.getPathTo(targetPose, isGather, isClimb));

        Pose2d flipPose;
        if (isGather) {
            flipPose = Locations.invert(targetPose);
        } else {
            flipPose = targetPose;
        }

        // rotate 90deg right for climb align
        // if (isClimb) {
        //     flipPose = targetPose.plus(new Transform2d(0, 0, Rotation2d.kCW_90deg));
        // }

        double startMovingThingsPosition;
        if (r.controlBoard.algaeModeT.getAsBoolean()) {
            startMovingThingsPosition = 0.0;
        } else {
            // coral
            if (r.controlBoard.selectedLevel == 4) {
                startMovingThingsPosition = waypoints.size() - 2.8;
            } else if (r.controlBoard.selectedLevel == 1) {
                startMovingThingsPosition = waypoints.size() - 2.8;
            } else {
                startMovingThingsPosition = waypoints.size() - 2.2;
            }
        }
        if (startMovingThingsPosition < 0) {
            startMovingThingsPosition = 0;
        }

        PathConstraints selectedConstraint;
        PathConstraints globalConstraint;
        if (r.controlBoard.algaeModeT.getAsBoolean() && !DriverStation.isAutonomous()) {
            selectedConstraint = finalConstraintsAlgae;
            globalConstraint = algaeGlobalConstraints;
        } else {
            if (isGather) {
                selectedConstraint = finalGatherConstraints;
            } else {
                selectedConstraint = finalConstraints;
            }
            globalConstraint = coralGlobalConstraints;
        }

        ConstraintsZone cz =
                new ConstraintsZone(waypoints.size() - 2, waypoints.size() - 1, selectedConstraint);
        RotationTarget rt = new RotationTarget(waypoints.size() - 2, flipPose.getRotation());
        EventMarker em =
                new EventMarker(
                        "InSlowDrivePhase", startMovingThingsPosition, waypoints.size() - 1);
        int minWaypoints = waypoints.size() - 3;
        if (minWaypoints < 0) minWaypoints = 0;
        EventMarker em2 = new EventMarker("InLocalPosePhase", minWaypoints, -1);

        // ChassisSpeeds botVel = r.drive.getVelocity();
        // double speed = Math.hypot(botVel.vxMetersPerSecond, botVel.vyMetersPerSecond);
        // Rotation2d angle = new Rotation2d(botVel.vxMetersPerSecond, botVel.vyMetersPerSecond);

        PathPlannerPath path =
                new PathPlannerPath(
                        waypoints,
                        List.of(rt),
                        Collections.emptyList(),
                        List.of(cz),
                        List.of(em, em2),
                        globalConstraint,
                        null,
                        new GoalEndState(0, flipPose.getRotation()),
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
            Logger.recordOutput("Odometry/Repaths", ++repaths);
        }
    }

    @Override
    public boolean isFinished() {
        return c.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        c.end(interrupted);
        r.drive.runVelocity(new ChassisSpeeds());
    }
}
