package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.util.Locations;
import java.util.function.Supplier;

public class PathfindingCommand extends Command {

    private RobotContainer r;
    private Supplier<Pose2d> poseSupplier;
    private boolean flip;
    private Command c;
    PathConstraints pathConstraints =
            new PathConstraints(3, 3, 2, 3); // vel, accel, rotvel, rotaccel

    public PathfindingCommand(RobotContainer r, Supplier<Pose2d> poseSupplier, boolean flip) {
        this.r = r;
        this.poseSupplier = poseSupplier;
        this.flip = flip;
    }

    @Override
    public void initialize() {
        Pose2d flipPose;
        if (flip) {
            flipPose = Locations.invert(poseSupplier.get());
        } else {
            flipPose = poseSupplier.get();
        }
        c = AutoBuilder.pathfindToPose(flipPose, pathConstraints);
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
