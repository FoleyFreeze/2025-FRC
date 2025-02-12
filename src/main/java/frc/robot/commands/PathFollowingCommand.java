package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.List;
import java.util.function.Supplier;

public class PathFollowingCommand extends Command {
    RobotContainer r;

    private Supplier<Pose2d> poseSupplier;
    private Command c;
    PathConstraints pathConstraints =
            new PathConstraints(1, 1, 1, 1); // vel, accel, rotvel, rotaccel

    public PathFollowingCommand(RobotContainer r, Supplier<Pose2d> poseSupplier) {
        this.poseSupplier = poseSupplier;
        this.r = r;
    }

    @Override
    public void initialize() {
        Pose2d targetPose = poseSupplier.get();

        Pose2d currentLocation = r.drive.getPose();

        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(currentLocation, targetPose);

        PathPlannerPath path =
                new PathPlannerPath(
                        waypoints,
                        pathConstraints,
                        new IdealStartingState(0, r.drive.getRotation()),
                        new GoalEndState(0, targetPose.getRotation()));

        c = AutoBuilder.followPath(path);
        c.initialize();
    }

    @Override
    public void execute() {
        c.execute();
    }

    @Override
    public boolean isFinished() {
        return c.isFinished();
    }

    @Override
    public void end(boolean interrupted) {
        c.end(interrupted);
    }
}
