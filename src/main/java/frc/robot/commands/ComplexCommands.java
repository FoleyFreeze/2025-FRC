package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

import com.google.gson.annotations.Until;

public class ComplexCommands {
    static double intakePower = 1.0;
    static double holdPower = 0.5;
    static double releasePower = -5;
    static double releaseTime = 0.5;

    static double intakeCompleteCurrent = 10;

    static double holdAlgaePower = 12;

    static double gatherPosition = 0;

    public static RobotContainer r;

    public static Command fancyCoralScore() {
        return null;
    }

    public static Command blindCoralScore() {
        return snapToAngle()
                .alongWith(noDriveScore());
    }

    public static Command blindGatherCoral() {
        return snapToAngle()
                .alongWith(noDriveGather());
    }

    public static Command noDriveScore() {
        return goToLoc(() -> r.controlBoard.getCoralLevelFromController(r.flysky))
                .alongWith(holdCoral())
                // gather trigger
                .until(r.flysky.leftTriggerSWE)
                .andThen(releaseCoral())
                //when complete schedule a new command to return the superstructure to the hold position
                .finallyDo(() -> goToLoc(() -> SuperstructureLocation.HOLD).schedule());
    }

    public static Command noDriveGather() {
        return goToLoc(() -> SuperstructureLocation.INTAKE).andThen(gatherCoral())
              //as soon as command finishes, come back up
               .finallyDo(() -> goToLoc(() -> SuperstructureLocation.HOLD).schedule());
    }

    public static Command gatherAlgae(){
        return goToLoc(() -> r.controlBoard.getAlgaeLevelFromController(r.flysky))
        .andThen(holdAlgae());
        //.until(() -> r.hand.getCurrent() > 50);
    }

    public static Command scoreAlgaeProc(){
        return goToLoc(() -> SuperstructureLocation.SCORE_PROCESSOR)
            .until(r.flysky.leftTriggerSWE)
            .andThen(releaseCoral());
    }

    public static Command scoreAlgaeNet(){
        return null;
    }

    public static Command holdAlgae(){
        return r.hand.setVoltage(holdAlgaePower);
    }

    // STES LOW POWER ON HAND TO KEEP CORAL IN
    public static Command holdCoral() {
        return r.hand.setVoltage(holdPower);
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
                .deadlineFor(
                        r.wrist.goTo(
                                p)) // this intenionally gets interrupted when the arm reaches its
                // dest
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
        return r.hand.setVoltage(intakePower)
            .until(() -> r.hand.getCurrent() > intakeCompleteCurrent)
        .andThen(new WaitCommand(0.2));
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
