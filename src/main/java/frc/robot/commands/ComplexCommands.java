package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.util.Locations;
import java.util.function.Supplier;

public class ComplexCommands {

    public static double holdPowerCoral = 0.4;
    static double releasePowerCoral23 = -2;
    static double releasePowerCoral4 = -2.5 - 1.5;
    public static double releasePowerCoral1 = -1.0;
    static double releaseTimeCoral1 = 0.3;
    static double releaseTimeCoral23 = 0.25;
    static double releaseTimeCoral4 = 0.05;

    public static double intakePowerCoral = 2.0;
    public static double intakeCurrentCoral = 15;
    static double intakeCoralTime = 0.8;

    static double intakeAlgaeTime = 0.2;
    static double intakeCurrentAlgae = 50;
    static double intakeAlgaePower = 3;
    static double holdAlgaePower = 2;
    static double releasePowerAlgae = -12;
    static double releaseTimeAlgae = 0.5;
    static double netAngle = 60;
    static double superSuckAlgae = 6;
    static double algaeHighLim = 90;
    static double algaeLowLim = 40;
    static double descoreAlgaePower = -4;
    static double stripTime = 2; // make smaller

    static double gatherPosition = 0;

    static double pulseGatherOn = 0.2;
    static double pulseGatherOff = 0.2;
    static double pulseGatherOffPwr = -0.05;

    public static RobotContainer r;

    // ALGAE COMMANDS

    public static Command stripAlgae() {
        Command c =
                // hand goes up
                r.wrist
                        .goTo(() -> SuperstructureLocation.ALGAE_DESCORE2_3)
                        // robot slides over to center
                        .andThen(DriveCommands.driveToPoint(r, r.controlBoard::getAlgaePathPose))
                        // arm angles down
                        .andThen(r.hand.setVoltageCmd(descoreAlgaePower))
                        // drops elevator on algae
                        .andThen(r.elevator.goTo(r.controlBoard::getAlgaeReefDSHeight))
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.ALGAE_DESCORE2_3))
                        .andThen(new WaitCommand(stripTime))
                        .andThen(r.hand.setVoltageCmd(0));

        c.setName("StripAlgae");
        return c;
    }

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
        SequentialCommandGroup launch = new SequentialCommandGroup();
        launch.addCommands(r.hand.setCurrentLimCmd(algaeHighLim));
        launch.addCommands(r.hand.setVoltageCmd(superSuckAlgae));
        launch.addCommands(new WaitCommand(0.1));
        launch.addCommands(
                forceGoToLoc(() -> SuperstructureLocation.NET)
                        .raceWith(
                                new WaitUntilCommand(() -> r.arm.getAngle().in(Degrees) < netAngle)
                                        .andThen(r.hand.setVoltageCmd(releasePowerAlgae))
                                        .andThen(new WaitCommand(0.5))));
        launch.setName("launch");

        Command c =
                goToLocAlgae(() -> SuperstructureLocation.PRENET)
                        .andThen(
                                new WaitUntilCommand(
                                        r.flysky.leftTriggerSWE.and(r.state.pathCompleteT)))
                        .andThen(launch)
                        .andThen(r.hand.stop())
                        .andThen(new RunCommand(() -> {}))
                        .finallyDo(
                                () -> {
                                    r.hand.setVoltage(0);
                                    r.hand.setCurrentLim(algaeLowLim);
                                });

        c.setName("ScoreAlgaeNet");
        return c;
    }

    public static Command scoreAlgaeProc() {
        Command c =
                goToLocAlgae(() -> SuperstructureLocation.SCORE_PROCESSOR)
                        .andThen(
                                new WaitUntilCommand(
                                        r.flysky.leftTriggerSWE.and(r.state.pathCompleteT)))
                        .andThen(releaseAlgae())
                        .andThen(new RunCommand(() -> {}));
        c.setName("ScoreAalgaeProc");
        return c;
    }

    public static Command gatherAlgae(boolean cmaMode) {
        Command c =
                goToLocAlgae(
                                () ->
                                        cmaMode
                                                ? r.controlBoard.getAlgaeReefHeight()
                                                : r.controlBoard.getAlgaeLevel())
                        .andThen(r.hand.setVoltageCmd(intakeAlgaePower))
                        // .until(() -> r.hand.getCurrent() > intakeCurrentAlgae)
                        .andThen(new InstantCommand(() -> r.state.setAlgae()))
                        .andThen(new RunCommand(() -> {})) // never end
                        .finallyDo(() -> r.hand.setVoltage(holdAlgaePower));
        c.setName("GatherAlgae");
        return c;
    }

    public static Command visionAlgaeGather() {
        Command c =
                DriveCommands.driveTo(r, r.controlBoard::getAlgaePathPose, false)
                        .raceWith(
                                new WaitUntilCommand(r.inSlowDrivePhase)
                                        .andThen(gatherAlgae(true)));
        c.setName("VisionAlgaeGather");
        return c;
    }

    public static Command blindAlgaeScore() {
        Command c =
                new ConditionalCommand(
                        scoreAlgaeNet(), scoreAlgaeProc(), () -> r.controlBoard.selectedLevel == 4);
        c.setName("BlindAlgaeScore");
        return c;
    }

    public static Command visionAlgaeScore() {
        Command c =
                new ConditionalCommand(
                        visionScoreAlgaeNet(),
                        visionScoreAlgaeProc(),
                        () -> r.controlBoard.selectedLevel == 4);

        c.setName("VisionAlgaeScore");
        return c;
    }

    public static Command visionScoreAlgaeNet() {
        Command c =
                DriveCommands.driveTo(r, () -> Locations.getNetPose(r.drive.getPose()), true)
                        .alongWith(new WaitCommand(0.25).andThen(scoreAlgaeNet()));
        c.setName("visionScoreAlgaeNet");
        return c;
    }

    public static Command visionScoreAlgaeProc() {
        Command c =
                DriveCommands.driveTo(r, Locations::getProcLoc, false)
                        .alongWith(new WaitCommand(0.25).andThen(scoreAlgaeProc()));
        // wait a sec to leave the reef before dropping elevator
        c.setName("VisionScoreAlgaeProc");
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
                                            r.hand.setVoltage(releasePowerCoral23);
                                            break;
                                        case 4:
                                            r.hand.setVoltage(releasePowerCoral4);
                                            break;
                                    }
                                })
                        .andThen(
                                new DynamicWaitCommand(
                                        () -> {
                                            switch (r.controlBoard.selectedLevel) {
                                                case 1:
                                                    return releaseTimeCoral1;
                                                case 2:
                                                case 3:
                                                    return releaseTimeCoral23;
                                                case 4:
                                                default:
                                                    return releaseTimeCoral4;
                                            }
                                        }))
                        .andThen(new InstantCommand(() -> r.state.clear()))
                        .andThen(r.hand.stop());
        c.setName("ReleaseCoral");
        return c;
    }

    // applies power to get rid of coral in auton
    public static Command releaseCoralAuton(int level) {

        Command c = // this doesnt work in teleop, but who cares, would need to be a supplier
                (level == 4
                                ? r.hand.setVoltageCmd(releasePowerCoral4)
                                : r.hand.setVoltageCmd(releasePowerCoral23))
                        .andThen(
                                new DynamicWaitCommand(
                                        () -> {
                                            switch (r.controlBoard.selectedLevel) {
                                                case 1:
                                                    return releaseTimeCoral1;
                                                case 2:
                                                case 3:
                                                    return releaseTimeCoral23;
                                                case 4:
                                                default:
                                                    return releaseTimeCoral4;
                                            }
                                        }))
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
                        .andThen(
                                new WaitUntilCommand(r.flysky.leftTriggerSWE.or(r.state.onTargetT)))
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
                                                                                        .and(
                                                                                                r.state
                                                                                                        .pathCompleteT)
                                                                                        .getAsBoolean())))
                                        .andThen(releaseCoral()))
                        .andThen(
                                new ConditionalCommand(
                                        stripAlgae(),
                                        new InstantCommand(),
                                        r.flysky.botRightSWHLo.negate()));
        c.setName("VisionCoralScore");
        return c;
    }

    // IMPORTANT/COMMONLY USED

    // moves elevator to a height with arm tucked up, then deploys arm
    public static Command goToLoc(Supplier<SuperstructureLocation> p) {
        Command notInGather =
                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                        .deadlineFor(r.wrist.goTo(() -> SuperstructureLocation.HOLD))
                        .andThen(r.elevator.goTo(p))
                        .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));

        Command inGather =
                r.elevator
                        .goTo(() -> SuperstructureLocation.PRE_INTAKE)
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.POST_INTAKE))
                        .andThen(r.wrist.goTo(() -> SuperstructureLocation.POST_INTAKE))
                        .andThen(
                                r.arm.goTo(() -> SuperstructureLocation.HOLD)
                                        .alongWith(r.wrist.goTo(() -> SuperstructureLocation.HOLD)))
                        .andThen(r.elevator.goTo(p))
                        .andThen(r.arm.goTo(p).alongWith(r.wrist.goTo(p)));

        Command c =
                new ConditionalCommand(
                        inGather,
                        notInGather,
                        () -> atLocation(SuperstructureLocation.INTAKE, true));

        // arm to 0, elevator move, arm out
        c.setName("GoToLoc");
        return c;
    }

    // no fancy protection logic, force to position
    public static Command forceGoToLoc(Supplier<SuperstructureLocation> p) {
        Command c = new ParallelCommandGroup(r.elevator.goTo(p), r.arm.goTo(p), r.wrist.goTo(p));
        c.setName("ForceGoToLoc");
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
                        toAlgaeFromCoral,
                        toAlgaeFromAlgae,
                        () -> r.arm.getAngle().in(Degrees) < -10);
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
                                                () ->
                                                        atLocation(
                                                                SuperstructureLocation.INTAKE,
                                                                true)),
                                        new ConditionalCommand(
                                                (new RunCommand(() -> {})),
                                                goToGather()
                                                        .andThen(
                                                                new RunCommand(
                                                                        () -> {})), // prevent
                                                // command from
                                                // ending
                                                r.controlBoard.algaeModeT), // coral/algae sw
                                        r.state.hasCoralT),
                                r.state.hasStopT.or(r.controlBoard.climbModeT))
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
                        .alongWith(r.elevator.goTo(() -> SuperstructureLocation.PRE_INTAKE))
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.PRE_INTAKE))
                        .andThen(r.wrist.goToReally(() -> SuperstructureLocation.PRE_INTAKE))
                        .andThen(r.elevator.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(r.arm.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(r.wrist.goTo(() -> SuperstructureLocation.INTAKE))
                        .andThen(new InstantCommand(() -> r.state.setCoral()));

        Command c =
                new ConditionalCommand(
                        rawGoTo(() -> SuperstructureLocation.INTAKE), // go directly to intake
                        toGather, // go to gather position via sequence
                        () -> atLocation(SuperstructureLocation.INTAKE, true));
        c.setName("GoToGather");
        return c;
    }

    // just goes to, nothing fancy, no protections
    public static Command rawGoTo(Supplier<SuperstructureLocation> loc) {
        return r.elevator.goTo(loc).alongWith(r.arm.goTo(loc)).alongWith(r.wrist.goTo(loc));
    }

    public static Command goToClimb() {
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(goToLoc(() -> SuperstructureLocation.HOLD));
        c.addCommands(r.hand.setVoltageCmd(releasePowerCoral1));
        c.addCommands(r.wrist.setVoltage(-0.7));
        c.addCommands(new WaitCommand(0.3));
        c.addCommands(r.wrist.setVoltage(-0.4));
        c.addCommands(r.arm.setVoltage(0));
        c.addCommands(r.hand.setVoltageCmd(0));

        c.setName("GoToClimb");
        return c;
    }

    public static Command leaveClimb() {
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.addCommands(r.wrist.setVoltage(0));
        c.addCommands(r.elevator.goTo(() -> SuperstructureLocation.HOLD));
        c.addCommands(r.arm.setVoltage(2));
        c.addCommands(new WaitUntilCommand(() -> r.arm.getAngle().in(Degrees) > -10));
        c.addCommands(r.arm.setVoltage(0));
        c.addCommands(r.wrist.goTo(() -> SuperstructureLocation.HOLD));
        c.addCommands(r.arm.goTo(() -> SuperstructureLocation.HOLD));

        c.setName("LeaveClimb");
        return c;
    }

    public static Command stopSuperstructure() {
        Command c = r.elevator.stop().alongWith(r.arm.stop()).alongWith(r.wrist.stop());
        // .alongWith(r.hand.stop());
        c.setName("StopSuperstructure");
        return c;
    }

    public static boolean atLocation(SuperstructureLocation loc, boolean extraTolerance) {
        return r.arm.atTarget(() -> loc, extraTolerance)
                && r.elevator.atTarget(() -> loc, extraTolerance)
                && r.wrist.atTarget(() -> loc, extraTolerance);
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
        c.addCommands(r.wrist.setVoltage(0.01));
        c.addCommands(r.hand.setVoltageCmd(releasePowerCoral23));
        c.addCommands(r.arm.goTo(() -> SuperstructureLocation.ZERO_WRIST));
        c.addCommands(r.elevator.goTo(() -> SuperstructureLocation.HOLD));
        c.addCommands(r.hand.stop());
        c.addCommands(r.wrist.setVoltage(-1));
        c.addCommands(new WaitCommand(2));
        c.addCommands(r.wrist.stop());
        c.addCommands(new InstantCommand(() -> r.wrist.rezeroAbsEnc()));
        c.addCommands(new WaitCommand(0.2));

        // c.addRequirements(r.elevator, r.arm, r.wrist, r.hand);
        c.setName("RezeroWrist");
        return c.withInterruptBehavior(InterruptionBehavior.kCancelIncoming);
    }
}
