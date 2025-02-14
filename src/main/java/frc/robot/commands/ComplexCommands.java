package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class ComplexCommands {
    static double intakePower = 1.0;
    static double holdPower = 0.5;
    static double releasePower = -5;
    static double releaseTime = 0.5;

    static double detectHandCurrent = 0;
    static double gatherPosition = 0;

    public static RobotContainer r;

    // notice: this is a hack
    static Supplier<SuperstructureLocation> upProvider =
            new Supplier<SuperstructureLocation>() {
                public SuperstructureLocation get() {
                    return SuperstructureLocation.LEVEL3;
                }
            };
    static Supplier<SuperstructureLocation> downProvider =
            new Supplier<SuperstructureLocation>() {
                public SuperstructureLocation get() {
                    return SuperstructureLocation.INTAKE;
                }
            };

    public static Command fancyCoralScore() {
        return null;
    }

    public static Command blindCoralScore() {
        return snapToAngle()
                .alongWith(goToLoc(() -> r.controlBoard.getLevelFromController(r.controller)))
                .alongWith(holdCoral())
                // gather trigger
                .until(r.controller.axisGreaterThan(2, 0))
                // .andThen(goToLoc(() -> null))
                .andThen(releaseCoral());
        // .andThen(goToLoc(() -> null))
    }

    public static Command blindGatherCoral() {
        return snapToAngle()
                .alongWith(goToLoc(() -> null))
                .andThen(gatherCoral())
                .andThen(goToLoc(() -> null));
    }

    public static Command noDriveScore() {
        return goToLoc(() -> r.controlBoard.getLevelFromController(r.controller))
                .alongWith(holdCoral())
                // gather trigger
                .until(r.controller.axisGreaterThan(2, 0))
                .andThen(releaseCoral());
    }

    public static Command noDriveGather() {
        return goToLoc(() -> SuperstructureLocation.INTAKE).andThen(gatherCoral());
    }

    // STES LOW POWER ON HAND TO KEEP CORAL IN
    public static Command holdCoral() {
        return r.hand.setVoltage(holdPower);
    }

    // lines up with target field element
    public static Command snapToAngle() {
        return DriveCommands.joystickDriveAtAngle(
                r.drive,
                () -> -r.controller.getLeftY(),
                () -> -r.controller.getLeftX(),
                r.controlBoard::getAlignAngle);
    }

    // moves elevator to a height with arm tucked up, then deploys arm
    public static Command goToLoc(Supplier<SuperstructureLocation> p) {
        return r.arm.goTo(() -> SuperstructureLocation.HOLD)
                .alongWith(r.wrist.goToLimit(p))
                .andThen(r.elevator.goTo(p))
                .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));
        // arm to 0, elevator move, arm out
    }

    // applies power to get rid of coral
    public static Command releaseCoral() {
        return r.hand.setVoltage(releasePower)
                .raceWith(new WaitCommand(releaseTime))
                .andThen(r.hand.stop());
    }

    public static Command gatherCoral() {
        return r.hand.setVoltage(intakePower);
    }

    public static Command stopSuperstructure() {
        return r.elevator.stop().alongWith(r.arm.stop()).alongWith(r.wrist.stop());
        // .alongWith(r.hand.stop());
    }

    public static Command zeroSuperstructure() {
        return new InstantCommand(
                () -> {
                    r.elevator.zero();
                    r.arm.zero();
                    r.wrist.zero();
                });
    }
}
