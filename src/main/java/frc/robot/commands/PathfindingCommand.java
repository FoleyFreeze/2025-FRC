package frc.robot.commands;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class PathfindingCommand extends Command {

    private RobotContainer r;
    private Supplier<Pose2d> poseSupplier;
    private Command c;
    PathConstraints pathConstraints =
            new PathConstraints(1, 1, 1, 1); // vel, accel, rotvel, rotaccel

    public PathfindingCommand(RobotContainer r, Supplier<Pose2d> poseSupplier) {
        this.r = r;
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void initialize() {
        c = AutoBuilder.pathfindToPose(poseSupplier.get(), pathConstraints);
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
