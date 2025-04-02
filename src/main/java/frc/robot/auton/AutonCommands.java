package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.auton.AutonSelection.GatherType;
import frc.robot.commands.ComplexCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.subsystems.controls.ControlBoard.ReefSticks;
import frc.robot.util.Locations;

public class AutonCommands {

    public static RobotContainer r;

    static double gatherStationWait = 0.0;
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

    public static Command scoreCoral(ReefSticks reefSticks, int level) {
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
                                                        r.hand.setVoltageCmd(-5),
                                                        () -> r.hand.checkForCoral())))
                        .andThen(ComplexCommands.releaseCoralAuton(level))
                        .alongWith(registerAutonCoralScore(reefSticks, level));
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
                .andThen(new WaitCommand(gatherStationWait))
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
