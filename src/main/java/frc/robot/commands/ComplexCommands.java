package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class ComplexCommands {

    static double holdPowerCoral = 0.4;
    static double releasePowerCoral = -6;
    static double releasePowerCoral4 = -4;
    static double releaseTimeCoral = 0.5;

    static double intakePowerCoral = 3.5;
    static double intakeCurrentCoral = 15;
    static double intakeCoralTime = 0.8;

    static double intakeAlgaeTime = 0.2;
    static double intakeCurrentAlgae = 50;
    static double intakeAlgaePower = 12;
    static double releasePowerAlgae = -12;
    static double releaseTimeAlgae = 0.5;

    static double gatherPosition = 0;

    public static RobotContainer r;

    // ALGAE COMMANDS

    public static Command releaseAlgae() {
        return r.hand.setVoltageCmd(releasePowerAlgae)
                .andThen(new WaitCommand(releaseTimeAlgae))
                .andThen(new InstantCommand(() -> r.state.clear()))
                .andThen(r.hand.stop());
    }

    public static Command holdAlgae() {
        return r.hand.setVoltageCmd(intakeAlgaePower);
    }

    public static Command scoreAlgaeNet() {
        return null;
    }

    public static Command scoreAlgaeProc() {
        return goToLoc(() -> SuperstructureLocation.SCORE_PROCESSOR)
                .andThen(new WaitUntilCommand(r.flysky.leftTriggerSWE))
                .andThen(releaseAlgae());
    }

    public static Command gatherAlgae() {
        return goToLoc(() -> r.controlBoard.getAlgaeLevelFromController(r.flysky))
                .andThen(holdAlgae())
                .until(() -> r.hand.getCurrent() > intakeCurrentAlgae)
                .andThen(new InstantCommand(() -> r.state.setAlgae()));
    }

    public static Command visionAlgaeGather() {
        return gatherAlgae();
    }

    public static Command visionAlgaeScore() {
        return scoreAlgaeProc();
    }

    // CORAL COMMANDS

    public static Command gatherCoral() {
        Command c =
                r.hand.setVoltageCmd(intakePowerCoral)
                        .andThen(new WaitCommand(0.5))
                        .andThen(
                                new WaitUntilCommand(
                                        () -> r.hand.getCurrent() > intakeCurrentCoral))
                        .andThen(new WaitCommand(intakeCoralTime))
                        .andThen(new InstantCommand(() -> r.state.setCoral()))
                        .finallyDo(() -> r.hand.setVoltage(holdPowerCoral));
        c.setName("GatherCoral");
        return c;
    }

    // applies power to get rid of coral
    public static Command releaseCoral() {
        return new ConditionalCommand(
                        r.hand.setVoltageCmd(releasePowerCoral4),
                        r.hand.setVoltageCmd(releasePowerCoral),
                        () -> r.controlBoard.selectedLevel == 4)
                .andThen(new WaitCommand(releaseTimeCoral))
                .andThen(new InstantCommand(() -> r.state.clear()))
                .andThen(r.hand.stop());
    }

    // applies power to get rid of coral in auton
    public static Command releaseCoralAuton(int level) {

        return (level == 4
                        ? r.hand.setVoltageCmd(releasePowerCoral4)
                        : r.hand.setVoltageCmd(releasePowerCoral))
                .andThen(new WaitCommand(releaseTimeCoral))
                .andThen(new InstantCommand(() -> r.state.clear()))
                .andThen(r.hand.stop());
    }

    // STES LOW POWER ON HAND TO KEEP CORAL IN
    public static Command holdCoral() {
        return r.hand.setVoltageCmd(holdPowerCoral);
    }

    public static Command noDriveGather() {
        Command c = goToGather().andThen(gatherCoral());
        c.setName("NoDriveGather");
        return c;
    }

    public static Command noDriveScore() {
        Command c =
                goToLoc(() -> r.controlBoard.getCoralLevel())
                        .alongWith(holdCoral())
                        // gather trigger
                        .andThen(new WaitUntilCommand(r.flysky.leftTriggerSWE))
                        .andThen(releaseCoral());
        c.setName("NoDriveScore");
        return c;
    }

    public static Command visionCoralGather() {
        // decides which coral station to use
        // drive there
        // gather
        return DriveCommands.driveTo(r, r.controlBoard::selectCoralStation, true)
                .alongWith(noDriveGather());
    }

    public static Command blindCoralScore() {
        return snapToAngle(r.controlBoard::getAlignAngle).alongWith(noDriveScore());
    }

    public static Command blindGatherCoral() {
        return snapToAngle(r.controlBoard::selectGatherAngle).alongWith(noDriveGather());
    }

    public static Command visionCoralScore() {
        return DriveCommands.driveTo(r, r.controlBoard::getPathPose, false)
                .andThen(goToLoc(r.controlBoard::getCoralLevel))
                .alongWith(holdCoral())
                // gather trigger
                .andThen(
                        new WaitUntilCommand(
                                () ->
                                        r.flysky.leftTriggerSWE.getAsBoolean()
                                                || RobotState.isAutonomous()))
                .andThen(releaseCoral());
    }

    // IMPORTANT/COMMONLY USED

    // moves elevator to a height with arm tucked up, then deploys arm
    public static Command goToLoc(Supplier<SuperstructureLocation> p) {
        return r.arm.goTo(() -> SuperstructureLocation.HOLD)
                .deadlineFor(r.wrist.goTo(() -> SuperstructureLocation.HOLD))
                .andThen(r.elevator.goTo(p))
                .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));
        // arm to 0, elevator move, arm out
    }

    // lines up with target field element
    public static Command snapToAngle(Supplier<Rotation2d> angle) {
        return DriveCommands.joystickDriveAtAngle(
                r,
                () -> -r.flysky.getLeftY(),
                () -> -r.flysky.getLeftX(),
                () -> -r.flysky.getRightX(),
                angle);
    }

    // go to hold, algae hold, or gather depending on what game pieces we have
    public static Command homeLogic() {
        return new ConditionalCommand(
                        new ConditionalCommand(
                                new RunCommand(() -> {}),
                                goToLoc(() -> SuperstructureLocation.HOLD),
                                () -> atLocation(SuperstructureLocation.INTAKE)),
                        new ConditionalCommand(
                                goToLoc(() -> SuperstructureLocation.ALGAE_LEVEL_2_3),
                                goToGather()
                                        .andThen(
                                                new RunCommand(
                                                        () -> {})), // prevent command from ending
                                r.flysky.topLeftSWA), // coral/algae sw
                        r.state.hasCoralT)
                .andThen(new WaitUntilCommand(() -> false));
    }

    // MINI COMMANDS

    public static Command goToGather() {
        Command toGather =
                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                        .alongWith(r.wrist.goToReally(() -> SuperstructureLocation.HOLD))
                        .andThen(r.wrist.goToReally(() -> SuperstructureLocation.HOLD_GATHER))
                        .alongWith(r.elevator.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(r.wrist.goToReally(() -> SuperstructureLocation.INTAKE));

        return new ConditionalCommand(
                new InstantCommand(), // do nothing
                toGather, // go to gather position
                () -> atLocation(SuperstructureLocation.INTAKE));
    }

    public static Command stopSuperstructure() {
        return r.elevator.stop().alongWith(r.arm.stop()).alongWith(r.wrist.stop());
        // .alongWith(r.hand.stop());
    }

    public static boolean atLocation(SuperstructureLocation loc) {
        return r.arm.atTarget(() -> loc)
                && r.elevator.atTarget(() -> loc)
                && r.wrist.atTarget(() -> loc);
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
