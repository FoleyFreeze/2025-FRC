package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.util.Locations;

public class ClimbCommands {

    public static Command autoDriveClimb(RobotContainer r) {
        // TODO: rotate robot if close but at wrong angle
        SequentialCommandGroup sg =
                new SequentialCommandGroup(
                        new ConditionalCommand(
                                new NewPathFinder(
                                        r, () -> Locations.getCageLocation(r), false, true),
                                new InstantCommand(),
                                () -> checkIfPathfindNeeded(r)),
                        new CmdDriveCageTraj(r));
        // r.climb.setClimbVoltage(12),
        // new WaitCommand(0.25),
        // r.climb.setClimbVoltage(0),
        // DriveCommands.joystickDriveFlysky(r)
        //         .raceWith(new WaitUntilCommand(r.flysky.rightTriggerSWG)),
        // r.climb.setClimbVoltage(12),
        // new WaitCommand(3),
        // r.climb.setClimbVoltage(0));

        sg.setName("AutoDriveClimb");
        return sg;
    }

    public static boolean checkIfPathfindNeeded(RobotContainer r) {
        Pose2d bot = r.drive.getGlobalPose();
        Pose2d target = Locations.getCageLocation(r);

        // only do the pathfind step
        return bot.minus(target).getTranslation().getNorm() > Units.inchesToMeters(40);
    }
}
