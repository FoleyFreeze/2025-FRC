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
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Locations;
import java.util.Collections;
import java.util.List;
import java.util.function.Supplier;

public class NewPathFinder extends Command {
    RobotContainer r;
    boolean isGather;

    private Supplier<Pose2d> poseSupplier;
    private Command c;

    // vel, accel, rotvel, rotaccel
    //PathConstraints globalConstraints = new PathConstraints(3.5, 5, 6, 4);
    PathConstraints globalConstraints = new PathConstraints(2, 2, 6, 4);
    PathConstraints finalConstraints = new PathConstraints(1.25, 1.5, 3, 2);
    PathConstraints finalConstraintsAlgae = new PathConstraints(2, 2, 3, 2);

    public NewPathFinder(RobotContainer r, Supplier<Pose2d> poseSupplier, boolean isGather) {
        this.poseSupplier = poseSupplier;
        this.r = r;
        this.isGather = isGather;
    }

    @Override
    public void initialize() {
        Pose2d targetPose = poseSupplier.get();

        List<Waypoint> waypoints = r.pathCache.getPathTo(targetPose, isGather);

        Pose2d flipPose;
        if (isGather) {
            flipPose = Locations.invert(targetPose);
        } else {
            flipPose = targetPose;
        }

        double startMovingThingsPosition;
        if (r.controlBoard.algaeModeT.getAsBoolean()) {
            startMovingThingsPosition = 0.25;
        } else {
            // coral
            if (r.controlBoard.selectedLevel == 4) {
                startMovingThingsPosition = waypoints.size() - 2.8;
            } else {
                startMovingThingsPosition = waypoints.size() - 2.2;
            }
        }
        if (startMovingThingsPosition < 0) {
            startMovingThingsPosition = 0;
        }

        PathConstraints selectedConstraint;
        if (r.controlBoard.algaeModeT.getAsBoolean()) {
            selectedConstraint = finalConstraintsAlgae;
        } else {
            selectedConstraint = finalConstraints;
        }

        ConstraintsZone cz =
                new ConstraintsZone(waypoints.size() - 2, waypoints.size() - 1, selectedConstraint);
        RotationTarget rt = new RotationTarget(waypoints.size() - 2, flipPose.getRotation());
        EventMarker em =
                new EventMarker(
                        "InSlowDrivePhase", startMovingThingsPosition, waypoints.size() - 1);

        PathPlannerPath path =
                new PathPlannerPath(
                        waypoints,
                        isGather ? Collections.emptyList() : List.of(rt),
                        Collections.emptyList(),
                        isGather ? Collections.emptyList() : List.of(cz),
                        isGather ? Collections.emptyList() : List.of(em),
                        globalConstraints,
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
