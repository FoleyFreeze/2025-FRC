package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import java.util.function.Supplier;

public class ComplexCommands {

    public static double holdPowerCoral = 0.4;
    static double releasePowerCoral = -3;
    static double releasePowerCoral4 = -4;
    public static double releasePowerCoral1 = -1.5;
    static double releaseTimeCoral = 0.5;

    public static double intakePowerCoral = 2.0;
    public static double intakeCurrentCoral = 15;
    static double intakeCoralTime = 0.8;

    static double intakeAlgaeTime = 0.2;
    static double intakeCurrentAlgae = 50;
    static double intakeAlgaePower = 3;
    static double holdAlgaePower = 2;
    static double releasePowerAlgae = -12;
    static double releaseTimeAlgae = 0.5;
    static double netAngle = 45;
    static double superSuckAlgae = 0;
    static double algaeHighLim = 0;
    static double algaeLowLim = 0;

    static double gatherPosition = 0;

    static double pulseGatherOn = 0.2;
    static double pulseGatherOff = 0.2;
    static double pulseGatherOffPwr = 0;

    public static RobotContainer r;

    // ALGAE COMMANDS

    public static Command releaseAlgae() {
        Command c =
                r.hand.setVoltageCmd(releasePowerAlgae)
                        .andThen(new WaitCommand(releaseTimeAlgae))
                        .andThen(new InstantCommand(() -> r.state.clear()))
                        .andThen(r.hand.stop());
        c.setName("ReleaseAlgae");
        return c;
    }

    public static Command holdAlgae() {
        Command c = r.hand.setVoltageCmd(holdAlgaePower);
        c.setName("HoldAlgae");
        return c;
    }

    public static Command scoreAlgaeNet() {
        Command c =
                goToLocAlgae(() -> SuperstructureLocation.PRENET)
                        .andThen(new WaitUntilCommand(r.flysky.leftTriggerSWE))
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.NET))
                                .alongWith(r.hand.setCurrentLimCmd(algaeHighLim)
                                           .andThen(r.hand.setVoltageCmd(superSuckAlgae))
                                           .andThen(new WaitUntilCommand(() -> r.arm.getAngle().in(Degrees) < netAngle))
                                           .andThen(r.hand.setVoltageCmd(releasePowerAlgae)))
                        .andThen(r.hand.stop())
                        .andThen(new RunCommand(() -> {}))
                        .finallyDo(() -> {r.hand.setVoltage(0);
                                          r.hand.setCurrentLim(algaeLowLim);});

        c.setName("ScoreAlgaeNet");
        return c;
    }

    public static Command scoreAlgaeProc() {
        Command c =
                goToLocAlgae(() -> SuperstructureLocation.SCORE_PROCESSOR)
                        .andThen(new WaitUntilCommand(r.flysky.leftTriggerSWE))
                        .andThen(releaseAlgae())
                        .andThen(new RunCommand(() -> {}));
        c.setName("ScoreAalgaeProc");
        return c;
    }

    public static Command gatherAlgae() {
        Command c =
                goToLocAlgae(() -> r.controlBoard.getAlgaeLevel())
                        .andThen(r.hand.setVoltageCmd(intakeAlgaePower))
                        // .until(() -> r.hand.getCurrent() > intakeCurrentAlgae)
                        .andThen(new InstantCommand(() -> r.state.setAlgae()))
                        .andThen(new RunCommand(() -> {})) // never end
                        .finallyDo(() -> r.hand.setVoltage(holdAlgaePower));
        c.setName("GatherAlgae");
        return c;
    }

    public static Command visionAlgaeGather() {
        Command c = gatherAlgae();
        c.setName("VisionAlgaeAlgae");
        return c;
    }

    public static Command visionAlgaeScore() {
        Command c =
                new ConditionalCommand(
                        scoreAlgaeNet(), scoreAlgaeProc(), () -> r.controlBoard.selectedLevel == 4);
        c.setName("VisionAlgaeScore");
        return c;
    }

    public static Command descoreAlgae() {
        Command c =
                goToLoc(r.controlBoard::getAlgaeDescoreLevel)
                        .andThen(r.hand.setVoltageCmd(4))
                        .andThen(new RunCommand(() -> {}))
                        .finallyDo(() -> r.hand.setVoltage(0));
        c.setName("descoreAlgae");
        return c;
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

    public static Command gatherCoral2() {
        Command c =
                pulseGather()
                        .finallyDo(
                                () -> {
                                    r.state.setCoral();
                                });

        c.setName("GatherCoral2");
        return c;
    }

    public static Command pulseGather() {
        Command c =
                new WaitCommand(pulseGatherOn)
                        .deadlineFor(r.hand.setVoltageCmd(intakePowerCoral))
                        .andThen(
                                new WaitCommand(pulseGatherOff)
                                        .deadlineFor(r.hand.setVoltageCmd(pulseGatherOffPwr)))
                        .repeatedly()
                        .finallyDo(() -> r.hand.setVoltage(holdPowerCoral));

        c.setName("PulseGather");
        return c;
    }

    // applies power to get rid of coral
    public static Command releaseCoral() {
        Command c =
                new InstantCommand(
                                () -> {
                                    switch (r.controlBoard.selectedLevel) {
                                        case 1:
                                            r.hand.setVoltage(releasePowerCoral1);
                                            break;
                                        case 2:
                                        case 3:
                                        default:
                                            r.hand.setVoltage(releasePowerCoral);
                                            break;
                                        case 4:
                                            r.hand.setVoltage(releasePowerCoral4);
                                            break;
                                    }
                                })
                        .andThen(new WaitCommand(releaseTimeCoral))
                        .andThen(new InstantCommand(() -> r.state.clear()))
                        .andThen(r.hand.stop());
        c.setName("ReleaseCoral");
        return c;
    }

    // applies power to get rid of coral in auton
    public static Command releaseCoralAuton(int level) {

        Command c = // TODO: this doesnt work, needs to be a supplier
                (level == 4
                                ? r.hand.setVoltageCmd(releasePowerCoral4)
                                : r.hand.setVoltageCmd(releasePowerCoral))
                        .andThen(new WaitCommand(releaseTimeCoral))
                        .andThen(new InstantCommand(() -> r.state.clear()))
                        .andThen(r.hand.stop());
        c.setName("ReleaseCoralAuton");
        return c;
    }

    // STES LOW POWER ON HAND TO KEEP CORAL IN
    public static Command holdCoral() {
        Command c = r.hand.setVoltageCmd(holdPowerCoral);
        c.setName("HoldCoral");
        return c;
    }

    public static Command noDriveGather() {
        Command c = goToGather().andThen(gatherCoral2());
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
        Command c =
                DriveCommands.driveTo(r, r.controlBoard::selectCoralStation, true)
                        .alongWith(noDriveGather());
        c.setName("VisionCoralGather");
        return c;
    }

    public static Command blindCoralScore() {
        Command snap = snapToAngle(r.controlBoard::getAlignAngle).alongWith(noDriveScore());
        Command justDrive = DriveCommands.joystickDriveFlysky(r).alongWith(noDriveScore());
        Command c =
                new ConditionalCommand(justDrive, snap, () -> r.controlBoard.selectedLevel == 1);
        c.setName("BlindCoralScore");
        return c;
    }

    public static Command blindGatherCoral() {
        Command c = snapToAngle(r.controlBoard::selectGatherAngle).alongWith(noDriveGather());
        c.setName("BlindGatherCoral");
        return c;
    }

    public static Command visionCoralScore() {
        Command c =
                DriveCommands.driveTo(r, r.controlBoard::getPathPose, false)
                        .raceWith(
                                new WaitUntilCommand(r.inSlowDrivePhase)
                                        .andThen(
                                                goToLoc(r.controlBoard::getCoralLevel)
                                                        .alongWith(holdCoral())
                                                        .andThen(
                                                                new WaitUntilCommand(
                                                                        () ->
                                                                                r.flysky
                                                                                        .leftTriggerSWE
                                                                                        .getAsBoolean())))
                                        .andThen(releaseCoral()));
        c.setName("VisionCoralScore");
        return c;
    }

    // IMPORTANT/COMMONLY USED

    // moves elevator to a height with arm tucked up, then deploys arm
    public static Command goToLoc(Supplier<SuperstructureLocation> p) {
        Command c =
                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                        .deadlineFor(r.wrist.goTo(() -> SuperstructureLocation.HOLD))
                        .andThen(r.elevator.goTo(p))
                        .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));
        // arm to 0, elevator move, arm out
        c.setName("GoToLoc");
        return c;
    }

    public static Command goToLocAlgae(Supplier<SuperstructureLocation> p) {
        Command toAlgaeFromCoral =
                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                        .deadlineFor(r.wrist.goTo(() -> SuperstructureLocation.HOLD))
                        .andThen(r.elevator.goTo(() -> SuperstructureLocation.HOLD_ALGAE))
                        .andThen(
                                r.elevator
                                        .goTo(p)
                                        .alongWith(r.arm.goTo(p).alongWith(r.wrist.goTo(p))));

        Command toAlgaeFromAlgae =
                r.arm.goTo(() -> SuperstructureLocation.HOLD_ALGAE)
                        .deadlineFor(r.wrist.goTo(() -> SuperstructureLocation.HOLD_ALGAE))
                        .andThen(r.elevator.goTo(p))
                        .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));

        Command c =
                new ConditionalCommand(
                        toAlgaeFromCoral, toAlgaeFromAlgae, () -> r.arm.getAngle().in(Degrees) < 0);
        c.setName("goToLocAlgae");
        return c;
    }

    // lines up with target field element
    public static Command snapToAngle(Supplier<Rotation2d> angle) {
        Command c =
                DriveCommands.joystickDriveAtAngle(
                        r,
                        () -> -r.flysky.getLeftY(),
                        () -> -r.flysky.getLeftX(),
                        () -> -r.flysky.getRightX(),
                        angle);
        c.setName("SnapToAngle");
        return c;
    }

    // go to hold, algae hold, or gather depending on what game pieces we have
    public static Command homeLogic() {
        Command c =
                new ConditionalCommand(
                                new RunCommand(() -> {}),
                                new ConditionalCommand(
                                        new ConditionalCommand(
                                                new RunCommand(() -> {}),
                                                goToLoc(() -> SuperstructureLocation.HOLD)
                                                        .andThen(new RunCommand(() -> {})),
                                                () -> atLocation(SuperstructureLocation.INTAKE)),
                                        new ConditionalCommand(
                                                goToLocAlgae(
                                                                () ->
                                                                        SuperstructureLocation
                                                                                .ALGAE_LEVEL_2_3)
                                                        .andThen(new RunCommand(() -> {})),
                                                goToGather()
                                                        .andThen(
                                                                new RunCommand(
                                                                        () -> {})), // prevent
                                                // command from
                                                // ending
                                                r.controlBoard.algaeModeT), // coral/algae sw
                                        r.state.hasCoralT),
                                r.state.hasStopT)
                        .andThen(new WaitUntilCommand(() -> false));
        c.setName("HomeLogic");
        return c;
    }

    // MINI COMMANDS

    public static Command goToGather() {
        Command toGather =
                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                        .alongWith(r.wrist.goToReally(() -> SuperstructureLocation.HOLD))
                        .andThen(
                                r.wrist
                                        .goToReally(() -> SuperstructureLocation.HOLD_GATHER)
                                        .raceWith(new WaitCommand(0.75)))
                        .alongWith(r.elevator.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(r.wrist.goToReally(() -> SuperstructureLocation.INTAKE));

        Command c =
                new ConditionalCommand(
                        new InstantCommand(), // do nothing
                        toGather, // go to gather position
                        () -> atLocation(SuperstructureLocation.INTAKE));
        c.setName("GoToGather");
        return c;
    }

    // just goes to, nothing fancy, no protections
    public static Command rawGoTo(Supplier<SuperstructureLocation> loc) {
        return r.elevator.goTo(loc).alongWith(r.arm.goTo(loc)).alongWith(r.wrist.goTo(loc));
    }

    public static Command goToClimb() {
        Command gatherFirst = goToGather().andThen(rawGoTo(() -> SuperstructureLocation.LOW_CLIMB));
        Command inGather = rawGoTo(() -> SuperstructureLocation.LOW_CLIMB);

        // if in gather, go straight to climb, otherwise go to gather first
        return new ConditionalCommand(
                inGather,
                gatherFirst,
                () -> r.elevator.getHeight().in(Inches) < 1 && r.arm.getAngle().in(Degrees) < -44);
    }

    public static Command stopSuperstructure() {
        Command c = r.elevator.stop().alongWith(r.arm.stop()).alongWith(r.wrist.stop());
        // .alongWith(r.hand.stop());
        c.setName("StopSuperstructure");
        return c;
    }

    public static boolean atLocation(SuperstructureLocation loc) {
        return r.arm.atTarget(() -> loc)
                && r.elevator.atTarget(() -> loc)
                && r.wrist.atTarget(() -> loc);
    }

    public static Command zeroSuperstructure() {
        Command c =
                new InstantCommand(
                        () -> {
                            r.elevator.zero();
                            r.arm.zero();
                            r.wrist.zero();
                        });
        c.setName("ZeroSuperstructure");
        return c;
    }

    public static Command rezeroWrist() {
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(r.hand.setVoltageCmd(releasePowerCoral));
        c.addCommands(goToLoc(() -> SuperstructureLocation.HOLD));
        c.addCommands(r.hand.stop());
        c.addCommands(r.wrist.setVoltage(-2));
        c.addCommands(new WaitCommand(2));
        c.addCommands(r.wrist.stop());
        c.addCommands(new InstantCommand(() -> r.wrist.resetPositionTo(-54)));
        c.addCommands(new PrintCommand("Rezeroed Wrist!"));

        // c.addRequirements(r.elevator, r.arm, r.wrist, r.hand);
        c.setName("RezeroWrist");
        return c.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
