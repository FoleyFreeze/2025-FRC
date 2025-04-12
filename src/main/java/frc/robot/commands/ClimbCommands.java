package frc.robot.commands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.util.Locations;

public class ClimbCommands {

    public static Command autoDriveClimb(RobotContainer r) {
        // TODO: rotate robot if close but at wrong angle
        SequentialCommandGroup sg =
                new SequentialCommandGroup(
                        // if not close drive close to the cage with pathfinder
                        new ConditionalCommand(
                                new NewPathFinder(
                                        r, () -> Locations.getCageLocation(r), false, true),
                                new InstantCommand(), // TODO: swap with a angle drive
                                () -> checkIfPathfindNeeded(r)),
                        // drive into cage
                        new CmdDriveCageTraj(r),
                        // do a little shake
                        DriveCommands.driveVel(r, new ChassisSpeeds(0, 0, 1))
                                .raceWith(new WaitCommand(0.2)),
                        DriveCommands.driveVel(r, new ChassisSpeeds(0, 0, -1))
                                .raceWith(new WaitCommand(0.4)),
                        DriveCommands.driveToAngle(
                                        r,
                                        () ->
                                                Locations.isBlue()
                                                        ? Rotation2d.kCW_90deg
                                                        : Rotation2d.kCCW_90deg)
                                .raceWith(new WaitCommand(0.3)),
                        new InstantCommand(() -> r.drive.runVelocity(new ChassisSpeeds())),
                        // never end until trigger released
                        new RunCommand(() -> {}));

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
