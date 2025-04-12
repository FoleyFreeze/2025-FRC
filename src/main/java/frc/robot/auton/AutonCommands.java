package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.RobotContainer;
import frc.robot.auton.AutonSelection.GatherType;
import frc.robot.auton.AutonSelection.ScoreType;
import frc.robot.commands.ComplexCommands;
import frc.robot.commands.DriveCommands;
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
                    r.controlBoard.autonGather = false;
                    r.controlBoard.autonScore = true;
                    r.controlBoard.autonTag = Locations.getTagId(reef);
                });
    }

    public static Command registerAutonCoralGather(GatherType station) {
        return new InstantCommand(
                () -> {
                    r.controlBoard.autonGather = true;
                    r.controlBoard.autonScore = false;
                    r.controlBoard.autonTag = Locations.getCoralStationTag(station);
                });
    }

    public static Command scoreCoral(ReefSticks reefSticks, ScoreType levelType) {
        int level;
        switch(levelType){
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
                                        () -> r.hand.checkForCoral()));

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
                                                        () -> r.hand.checkForCoral())))
                        .andThen(ComplexCommands.releaseCoralAuton(level))
                        .alongWith(registerAutonCoralScore(reefSticks, level))
                        .raceWith(r.leds.setLEDMode(LED_MODES.BLUE));
        c.setName("scoreCoral");
        return c;
    }

    // public static Command scoreAlgaeProcessor() {
    //     return DriveCommands.driveToAuto(r, () -> Locations.getProcLoc(), false)
    //             .andThen(ComplexCommands.releaseAlgae())
    //             .andThen(ComplexCommands.goToLoc(() -> SuperstructureLocation.INTAKE));
    // }

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

    // public static Command gather() {
    //     return r.hand.setVoltageCmd(ComplexCommands.intakePowerCoral)
    //             .until(() -> r.hand.getCurrent() > ComplexCommands.intakeCurrentCoral)
    //             /*.raceWith(new WaitCommand(1))*/
    //             // for sim
    //             .andThen(new WaitCommand(gatherPowerForExtraTime))
    //             .finallyDo(() -> r.hand.setVoltage(ComplexCommands.holdPowerCoral));
    // }

    // public static Command scoreAlgaeNet(int location) {}

    // public static Command algaeFloorGather() {}

    // public static Command algaeReefGather() {}

}
