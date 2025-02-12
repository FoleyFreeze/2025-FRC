package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;

public class PathfindingCommand extends Command{

    private Supplier<Pose2d> poseSupplier;
    private Command c;
    PathConstraints pathConstraints = new PathConstraints(1, 1, 1, 1);//vel, accel, rotvel, rotaccel

    public PathfindingCommand(Supplier<Pose2d> poseSupplier){
        this.poseSupplier = poseSupplier;
    }

    @Override
    public void initialize(){
        c = AutoBuilder.pathfindToPose(poseSupplier.get(), pathConstraints);
        c.initialize();
    }

    @Override
    public void execute(){
        c.execute();
    }

    @Override
    public boolean isFinished(){
        return c.isFinished();
    }

    @Override
    public void end(boolean interrupted){
        c.end(interrupted);
    }
}