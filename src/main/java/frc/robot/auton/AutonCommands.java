package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.commands.ComplexCommands;
import frc.robot.commands.DriveCommands;
import frc.robot.commands.SuperstructureLocation;
import frc.robot.subsystems.controls.ControlBoard.ReefSticks;
import frc.robot.util.Locations;

public class AutonCommands {

    public static RobotContainer r;

    public static Command scoreCoral(ReefSticks reefSticks, int level) {
        return DriveCommands.driveTo(r, () -> Locations.getReefLocation(reefSticks), false)
                .andThen(ComplexCommands.goToLoc(() -> r.controlBoard.getLevelLocation(level)))
                .andThen(ComplexCommands.releaseCoralAuton(level))
                .andThen(ComplexCommands.goToLoc(() -> SuperstructureLocation.INTAKE));
    }

    public static Command scoreAlgaeProcessor() {
        return DriveCommands.driveTo(r, () -> Locations.getProcLoc(), false)
                .andThen(ComplexCommands.releaseAlgae())
                .andThen(ComplexCommands.goToLoc(() -> SuperstructureLocation.INTAKE));
    }

    public static Command coralStationGather(Pose2d station) {
        return DriveCommands.driveTo(r, () -> station, true)
                .alongWith(ComplexCommands.noDriveGather())
                .andThen(ComplexCommands.goToLoc(() -> SuperstructureLocation.HOLD));
    }

    // public static Command scoreAlgaeNet(int location) {}

    // public static Command algaeFloorGather() {}

    // public static Command algaeReefGather() {}

}
