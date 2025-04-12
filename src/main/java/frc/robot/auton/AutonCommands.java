package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.auton.AutonSelection.GatherType;
import frc.robot.auton.AutonSelection.ScoreType;
import frc.robot.commands.BargePathFinder;
import frc.robot.commands.ComplexCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.PathFollowingCommand;
import frc.robot.commands.SuperstructureLocation;
import frc.robot.subsystems.LEDs.LED.LED_MODES;
import frc.robot.subsystems.controls.ControlBoard.ReefSticks;
import frc.robot.util.Locations;

public class AutonCommands {

    public static RobotContainer r;

    static double gatherStationWait = 0.8;
    static double gatherPowerForExtraTime = 0.7;

    public static Command registerAutonCoralScore(ReefSticks reef, int level) {
        return new InstantCommand(
                () -> {
                    r.controlBoard.autonCoralGather = false;
                    r.controlBoard.autonCoralScore = true;
                    r.controlBoard.autonTag = Locations.getTagId(reef);
                });
    }

    public static Command registerAutonCoralGather(GatherType station) {
        return new InstantCommand(
                () -> {
                    r.controlBoard.autonCoralGather = true;
                    r.controlBoard.autonCoralScore = false;
                    r.controlBoard.autonTag = Locations.getCoralStationTag(station);
                });
    }

    public static Command registerAutonAlgaeScore() {
        return new InstantCommand(
                () -> {
                    r.controlBoard.autonCoralGather = false;
                    r.controlBoard.autonCoralScore = false;
                    r.controlBoard.autonAlgaeGather = false;
                    r.controlBoard.autonAlgaeScore = true;
                });
    }

    public static Command registerAutonAlgaeGather() {
        return new InstantCommand(
                () -> {
                    r.controlBoard.autonCoralGather = false;
                    r.controlBoard.autonCoralScore = false;
                    r.controlBoard.autonAlgaeGather = true;
                    r.controlBoard.autonAlgaeScore = false;
                });
    }

    public static Command scoreCoral(ReefSticks reefSticks, ScoreType levelType) {
        return scoreCoral(reefSticks, levelType, false);
    }

    public static Command scoreCoral(ReefSticks reefSticks, ScoreType levelType, boolean isFirst) {
        int level;
        switch (levelType) {
            case ONE:
                level = 1;
                break;
            case TWO:
                level = 2;
                break;
            case THREE:
                level = 3;
                break;
            case FOUR:
                level = 4;
                break;
            default:
                level = 4;
        }

        Command waitUntilPathCompleteThenScore =
                new WaitUntilCommand(r.state.pathCompleteT)
                        .andThen(
                                new ConditionalCommand(
                                        ComplexCommands.goToLoc(
                                                () -> r.controlBoard.getLevelLocation(level)),
                                        r.hand.setVoltageCmd(-5),
                                        () -> r.hand.checkForCoral() || isFirst));

        Command c =
                DriveCommands.driveToAuto(r, () -> Locations.getReefLocation(reefSticks), false)
                        .alongWith(
                                ComplexCommands.pulseGather()
                                        .until(r.inSlowDrivePhase)
                                        .andThen(
                                                new ConditionalCommand(
                                                        ComplexCommands.goToLoc(
                                                                () ->
                                                                        r.controlBoard
                                                                                .getLevelLocation(
                                                                                        level)),
                                                        waitUntilPathCompleteThenScore,
                                                        () -> r.hand.checkForCoral() || isFirst)))
                        .andThen(ComplexCommands.releaseCoralAuton(level))
                        .alongWith(registerAutonCoralScore(reefSticks, level))
                        .raceWith(r.leds.setLEDMode(LED_MODES.BLUE));
        c.setName("scoreCoral");
        return c;
    }

    public static Command coralStationGather(Pose2d station, GatherType stationId) {
        return DriveCommands.driveToAuto(r, () -> station, true)
                .deadlineFor(
                        DriveCommands.waitUntilClose(r, () -> station, 30)
                                .andThen(r.leds.setLEDMode(LED_MODES.GREEN)))
                // only abort the wait early if there is coral
                .andThen(new WaitCommand(gatherStationWait).raceWith(r.hand.hasCoralInBucket()))
                .deadlineFor(ComplexCommands.goToGather().andThen(ComplexCommands.pulseGather()))
                // .raceWith(r.hand.hasCoralInBucket()) // abort early once the coral is there
                .alongWith(registerAutonCoralGather(stationId));
    }

    public static Command algaeGather(ReefSticks stick) {
        Command c =
                DriveCommands.driveToAuto(r, () -> Locations.getAlgaeReefLocation(stick), false)
                        .raceWith(
                                new WaitUntilCommand(r.inSlowDrivePhase)
                                        .andThen(autonHandGatherAlgae(stick)))
                        .andThen(new WaitCommand(0.1))
                        .andThen(
                                new PathFollowingCommand(
                                                r, () -> r.pathCache.closestWaypoint(), true)
                                        .deadlineFor(
                                                new WaitCommand(0.5)
                                                        .andThen(ComplexCommands.goToAlgaeHold())))
                        .alongWith(registerAutonAlgaeGather());

        // run the supercycle if we just scored on that stick
        c = new ConditionalCommand(supercycle(stick), c, () -> atTarget(stick));

        c.setName("AutonAlgaeGather");
        return c;
    }

    public static Command autonHandGatherAlgae(ReefSticks stick) {
        Command c =
                ComplexCommands.goToLocAlgae(() -> r.controlBoard.getAlgaeReefHeight(stick))
                        .andThen(r.hand.setVoltageCmd(ComplexCommands.intakeAlgaePower))
                        .andThen(new InstantCommand(() -> r.state.setAlgae()))
                        .andThen(new RunCommand(() -> {}))
                        .finallyDo(() -> r.hand.setVoltage(ComplexCommands.holdAlgaePower));
        c.setName("AutonGatherAlgae");
        return c;
    }

    public static Command algaeScore(ScoreType score) {
        switch (score) {
            case NET:
            default:
                return scoreAlgaeNet().alongWith(registerAutonAlgaeScore());

            case PROC:
                return scoreAlgaeProc().alongWith(registerAutonAlgaeScore());
        }
    }

    public static Command scoreAlgaeNet() {
        Command c =
                new SequentialCommandGroup(
                                ComplexCommands.goToAlgaeHold(),
                                new WaitUntilCommand(r.state.pathCompleteT.or(r.shootForNet)),
                                ComplexCommands.netLaunch(false))
                        .deadlineFor(new BargePathFinder(r));

        c.setName("AutonScoreAlgaeNet");
        return c;
    }

    public static Command scoreAlgaeProc() {
        // TODO: if we want auto processor
        // will also need to split r.cb.autonAlgaeScore between barge and proc
        return new InstantCommand();
    }

    public static Command supercycle(ReefSticks stick) {
        final boolean[] hasGathered = new boolean[1];

        SequentialCommandGroup moveArmlol =
                new SequentialCommandGroup(
                        r.arm.goTo(() -> SuperstructureLocation.HOLD)
                                .alongWith(
                                        r.elevator.goTo(
                                                () -> r.controlBoard.getAlgaeReefHeight(stick))),
                        r.wrist
                                .goTo(() -> SuperstructureLocation.ALGAE_LEVEL_2_3)
                                .alongWith(
                                        r.arm.goTo(() -> SuperstructureLocation.ALGAE_LEVEL_2_3)));

        SequentialCommandGroup mainSQ =
                new SequentialCommandGroup(
                        new InstantCommand(() -> hasGathered[0] = false),
                        r.hand.setVoltageCmd(ComplexCommands.intakeAlgaePower),
                        DriveCommands.driveToPoint(
                                        r,
                                        Locations.supercycleOffset(
                                                () -> Locations.getAlgaeReefLocation(stick)),
                                        false)
                                .alongWith(moveArmlol),
                        DriveCommands.driveToPoint(
                                r, () -> Locations.getAlgaeReefLocation(stick), false),
                        new InstantCommand(
                                () -> {
                                    hasGathered[0] = true;
                                    r.state.setAlgae();
                                }),
                        new WaitCommand(0.1),
                        new PathFollowingCommand(r, () -> r.pathCache.closestWaypoint(), true)
                                .deadlineFor(
                                        new WaitCommand(0.5)
                                                .andThen(ComplexCommands.goToAlgaeHold())));

        Command c = mainSQ.finallyDo(() -> r.controlBoard.tempScoreAlgae = hasGathered[0]);

        c.setName("SupercycleGather");
        return c;
    }

    public static boolean atTarget(ReefSticks stick) {
        double dist =
                r.drive
                        .getLocalPose()
                        .getTranslation()
                        .getDistance(Locations.getAlgaeReefLocation(stick).getTranslation());

        return dist < Units.inchesToMeters(18);
    }
}
