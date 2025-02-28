package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class JoystickAbort extends Command{
    RobotContainer r;
    double threshold = 0.1;

    public JoystickAbort(RobotContainer r){
        this.r = r;
    }

    @Override
    public boolean isFinished(){
        return Math.abs(r.flysky.getLeftX()) > threshold ||
               Math.abs(r.flysky.getLeftY()) > threshold ||
               Math.abs(r.flysky.getRightX()) > threshold;
    }
}
