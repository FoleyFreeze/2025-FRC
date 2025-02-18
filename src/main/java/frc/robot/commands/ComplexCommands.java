package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class ComplexCommands {

    static double holdPowerCoral = 0.6;
    static double releasePowerCoral = -5;
    static double releaseTimeCoral = 0.5;

    static double intakePowerCoral = 3.5;
    static double intakeCurrentCoral = 100;
    static double intakeCoralTime = 0.2;

    static double intakeAlgaeTime = 0.2;
    static double intakeCurrentAlgae = 50;
    static double intakeAlgaePower = 12;
    static double releasePowerAlgae = -12;
    static double releaseTimeAlgae = 0.5;

    static double gatherPosition = 0;

    public static RobotContainer r;

    public static Command fancyCoralScore() {
        return null;
    }

    public static Command blindCoralScore() {
        return snapToAngle().alongWith(noDriveScore());
    }

    public static Command blindGatherCoral() {
        return snapToAngle().alongWith(noDriveGather());
    }

    public static Command noDriveScore() {
        return goToLoc(() -> r.controlBoard.getCoralLevelFromController(r.flysky))
                .alongWith(holdCoral())
                // gather trigger
                .until(r.flysky.leftTriggerSWE)
                .andThen(releaseCoral())
                // when complete schedule a new command to return the superstructure to the hold
                // position
                .finallyDo(ComplexCommands::goHome);
    }

    public static Command noDriveGather() {
        return goToGather()
                .andThen(gatherCoral())
                // as soon as command finishes, come back up
                .finallyDo(ComplexCommands::goHome);
    }

    public static Command gatherAlgae() {
        return goToLoc(() -> r.controlBoard.getAlgaeLevelFromController(r.flysky))
                .andThen(holdAlgae());
        // .until(() -> r.hand.getCurrent() > 50);
    }

    public static Command scoreAlgaeProc() {
        return goToLoc(() -> SuperstructureLocation.SCORE_PROCESSOR)
                .andThen(new WaitUntilCommand(r.flysky.leftTriggerSWE))
                .andThen(releaseAlgae())
                .andThen(ComplexCommands::goHome);
    }

    public static Command scoreAlgaeNet() {
        return null;
    }

    public static Command holdAlgae() {
        return r.hand.setVoltage(intakeAlgaePower);
    }

    public static Command releaseAlgae() {
        return r.hand.setVoltage(releasePowerAlgae)
                .andThen(new WaitCommand(releaseTimeAlgae))
                .andThen(r.hand.stop())
                // TODO: does this help?
                .withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }

    // STES LOW POWER ON HAND TO KEEP CORAL IN
    public static Command holdCoral() {
        return r.hand.setVoltage(holdPowerCoral);
    }

    // applies power to get rid of coral
    public static Command releaseCoral() {
        return r.hand.setVoltage(releasePowerCoral)
                .raceWith(new WaitCommand(releaseTimeCoral))
                .andThen(r.hand.stop());
    }

    public static Command gatherCoral() {
        return r.hand.setVoltage(intakePowerCoral)
                .andThen(new WaitCommand(0.1))
                .until(() -> r.hand.getCurrent() > intakeCurrentCoral)
                .andThen(new WaitCommand(intakeCoralTime))
                .finallyDo(() -> holdCoral().schedule());
    }

    // lines up with target field element
    public static Command snapToAngle() {
        return DriveCommands.joystickDriveAtAngle(
                r.drive,
                () -> -r.flysky.getLeftY(),
                () -> -r.flysky.getLeftX(),
                r.controlBoard::getAlignAngle);
    }

    // moves elevator to a height with arm tucked up, then deploys arm
    public static Command goToLoc(Supplier<SuperstructureLocation> p) {
        return r.arm.goTo(() -> SuperstructureLocation.HOLD)
                .deadlineFor(r.wrist.goTo(() -> SuperstructureLocation.HOLD))
                .andThen(r.elevator.goTo(p))
                .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));
        // arm to 0, elevator move, arm out
    }

    public static Command goToGather() {
        return r.arm.goTo(() -> SuperstructureLocation.HOLD)
                .alongWith(r.wrist.goToReally(() -> SuperstructureLocation.HOLD))
                .andThen(r.wrist.goToReally(() -> SuperstructureLocation.HOLD_GATHER))
                .andThen(r.arm.goTo(() -> SuperstructureLocation.INTAKE))
                .andThen(r.wrist.goToReally(() -> SuperstructureLocation.INTAKE));
    }

    public static void goHome() {
        goToLoc(() -> SuperstructureLocation.HOLD).schedule();
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
