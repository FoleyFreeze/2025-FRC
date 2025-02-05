package frc.robot.commands;

import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class ComplexCommands {
    static double startupTime = 0;
    static double intakePower = 0;
    static double detectHandCurrent  = 0;
    static double gatherPosition = 0;

    RobotContainer r = RobotContainer.getInstance();

    public static Command fancyCoralScore() {
        return null;
    }

    

    public static Command blindCoralScore() {
            return snapToAngle()
            .alongWith(goToLoc(() -> null))
            .alongWith(holdCoral())
            ///wait for trigger
            .andThen(goToLoc(() -> null))
            .andThen(releaseCoral())
            .andThen(goToLoc(() -> null))
            .alongWith(holdCoral());
    }

    public static Command holdCoral(){
        return null;
    }

    public static Command stayStill(){
        return null;
    }

    public static Command snapToAngle(){
        return null;
    }

    public static Command goToLoc(Supplier<SuperstructureLocation> p){
        return null;
    }

    public static Command releaseCoral(){
        return null;
    }
    
}
