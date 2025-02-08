package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class ComplexCommands {
    static double intakePower = 0;
    static double holdPower = 0;
    static double releasePower = 0;

    static double detectHandCurrent = 0;
    static double gatherPosition = 0;

    static RobotContainer r = RobotContainer.getInstance();

    //notice: this is a hack
    static Supplier<SuperstructureLocation> upProvider = new Supplier<SuperstructureLocation>(){
        public SuperstructureLocation get(){
            return SuperstructureLocation.LEVEL3;
        }
    };
    static Supplier<SuperstructureLocation> downProvider = new Supplier<SuperstructureLocation>(){
        public SuperstructureLocation get(){
            return SuperstructureLocation.INTAKE;
        }
    };

    public static Command fancyCoralScore() {
        return null;
    }

    public static Command blindCoralScore() {
        return snapToAngle()
                .alongWith(goToLoc(() -> null))
                .alongWith(holdCoral())
                /// wait for trigger
                .andThen(goToLoc(() -> null))
                .andThen(releaseCoral())
                .andThen(goToLoc(() -> null))
                .alongWith(holdCoral());
    }

    public static Command blindGatherCoral() {
        return snapToAngle()
                .alongWith(goToLoc(() -> null))
                .andThen(gatherCoral())
                .andThen(goToLoc(() -> null));
    }

    // STES LOW POWER ON HAND TO KEEP CORAL IN
    public static Command holdCoral() {
        return r.hand.setVoltage(holdPower);
    }

    // lines up with target field element
    public static Command snapToAngle() {
        return null;
    }

    // moves elevator to a height with arm tucked up, then deploys arm
    public static Command goToLoc(Supplier<SuperstructureLocation> p) {
        return r.elevator.goTo(p);
        //arm to 0, elevator move, arm out
    }

    // applies power to get rid of coral
    public static Command releaseCoral() {
        return r.hand.setVoltage(releasePower);
    }

    public static Command gatherCoral() {
        return r.hand.setVoltage(intakePower);
    }
}
