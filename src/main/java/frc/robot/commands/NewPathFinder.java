package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Locations;
import java.util.List;
import java.util.function.Supplier;

public class NewPathFinder extends Command {
    RobotContainer r;
    boolean flipRobot;

    private Supplier<Pose2d> poseSupplier;
    private Command c;
    PathConstraints pathConstraints =
            new PathConstraints(2, 3, 2, 2); // vel, accel, rotvel, rotaccel

    public NewPathFinder(RobotContainer r, Supplier<Pose2d> poseSupplier, boolean flipRobot) {
        this.poseSupplier = poseSupplier;
        this.r = r;
        this.flipRobot = flipRobot;
    }

    @Override
    public void initialize() {
        Pose2d targetPose = poseSupplier.get();

        List<Waypoint> waypoints = r.pathCache.getPathTo(targetPose);

        Pose2d flipPose;
        if (flipRobot) {
            flipPose = Locations.invert(targetPose);
        } else {
            flipPose = targetPose;
        }

        PathPlannerPath path =
                new PathPlannerPath(
                        waypoints,
                        pathConstraints,
                        new IdealStartingState(0, flipPose.getRotation()),
                        new GoalEndState(0, flipPose.getRotation()));

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
        r.drive.runVelocity(new ChassisSpeeds());
    }
}
