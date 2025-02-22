package frc.robot.subsystems.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.SuperstructureLocation;
import frc.robot.util.Locations;
import java.util.List;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ControlBoard {

    public Joystick cb;
    public Joystick cb2;
    public LoggedDashboardChooser<Integer> level;
    public LoggedDashboardChooser<ReefSticks> letter;
    public LoggedDashboardChooser<Boolean> station;
    public LoggedDashboardChooser<Boolean> climbMode;

    RobotContainer r;

    public ControlBoard(RobotContainer r) {
        this.r = r;

        cb = new Joystick(1);
        cb2 = new Joystick(2);
        level = new LoggedDashboardChooser<>("Level");
        letter = new LoggedDashboardChooser<>("Letter");
        station = new LoggedDashboardChooser<>("Station");
        climbMode = new LoggedDashboardChooser<>("climbMode");

        level.addDefaultOption("1", 1);
        level.addOption("2", 2);
        level.addOption("3", 3);
        level.addOption("4", 4);

        letter.addDefaultOption("A", ReefSticks.A);
        letter.addOption("B", ReefSticks.B);
        letter.addOption("C", ReefSticks.C);
        letter.addOption("D", ReefSticks.D);
        letter.addOption("E", ReefSticks.E);
        letter.addOption("F", ReefSticks.F);
        letter.addOption("G", ReefSticks.G);
        letter.addOption("H", ReefSticks.H);
        letter.addOption("I", ReefSticks.I);
        letter.addOption("J", ReefSticks.J);
        letter.addOption("K", ReefSticks.K);
        letter.addOption("L", ReefSticks.L);

        station.addDefaultOption("Left", true);
        station.addOption("Right", false);

        climbMode.addDefaultOption("Off", false);
        climbMode.addOption("On", true);
    }

    /*
    public Trigger climbSW = new Trigger(() -> cb.getRawButton(3));
    public Trigger shift = new Trigger(() -> cb.getRawButton(3));
    public Trigger levelSW = new Trigger(() -> cb.getRawButton(3));
    public Trigger  = new Trigger(() -> cb.getRawButton(3));
    */

    public static enum ReefSticks {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L;
    }

    public ReefSticks selectedReefPos;
    public int selectedLevel;
    public boolean selectedStation;
    public boolean selectedClimbMode;

    public void periodic() {
        selectedReefPos = letter.get();
        selectedLevel = level.get();
        selectedStation = station.get();
        selectedClimbMode = climbMode.get();
    }

    public Trigger climbModeT = new Trigger(() -> selectedClimbMode);

    // left means true
    public Pose2d selectCoralStation() {
        if (selectedStation) {
            return Locations.getLeftGatherStation();
        } else {
            return Locations.getRightGatherStation();
        }
    }

    public Rotation2d selectGatherAngle() {
        if (selectedStation) {
            return Locations.getLeftGatherStation().getRotation().plus(Rotation2d.k180deg);
        } else {
            return Locations.getRightGatherStation().getRotation().plus(Rotation2d.k180deg);
        }
    }

    public Pose2d getPathPose() {
        return Locations.getReefLocation(selectedReefPos);
    }

    public Rotation2d getAlignAngle() {
        Rotation2d scoringPosition;
        switch (selectedReefPos) {
            case A:
            case B:
                scoringPosition = Rotation2d.fromDegrees(0);
                break;
            case C:
            case D:
                scoringPosition = Rotation2d.fromDegrees(60);
                break;
            case E:
            case F:
                scoringPosition = Rotation2d.fromDegrees(120);
                break;
            case G:
            case H:
                scoringPosition = Rotation2d.fromDegrees(180);
                break;
            case I:
            case J:
                scoringPosition = Rotation2d.fromDegrees(240);
                break;
            case K:
            case L:
                scoringPosition = Rotation2d.fromDegrees(300);
                break;
            default:
                scoringPosition = Rotation2d.fromDegrees(0);
        }
        if (Locations.isBlue()) {
            return scoringPosition;
        } else {
            return scoringPosition.plus(Rotation2d.fromDegrees(180));
        }
    }

    public SuperstructureLocation getCoralLevel() {
        return getLevelLocation(selectedLevel);
    }

    public SuperstructureLocation getCoralLevelFromController(Flysky controller) {
        int level = 0;
        if (controller.botRightSWHLo.getAsBoolean()) {
            level = 2;
        } else if (controller.botRightSWHHi.getAsBoolean()) {
            level = 4;
        } else {
            level = 3;
        }

        return getLevelLocation(level);
    }

    public SuperstructureLocation getAlgaeLevelFromController(Flysky controller) {
        if (controller.botRightSWHLo.getAsBoolean()) {
            return SuperstructureLocation.ALGAE_LEVEL_2_3;
        } else if (controller.botRightSWHHi.getAsBoolean()) {
            return SuperstructureLocation.ALGAE_LEVEL_3_4;
        } else {
            return SuperstructureLocation.ALGAE_LEVEL_2_3;
        }
    }

    public SuperstructureLocation getLevelLocation(int reefLevel) {
        switch (reefLevel) {
            case 1:
                return SuperstructureLocation.LEVEL1;
            case 2:
                return SuperstructureLocation.LEVEL2;
            case 3:
                return SuperstructureLocation.LEVEL3;
            case 4:
                return SuperstructureLocation.LEVEL4;
            default:
                return SuperstructureLocation.INTAKE;
        }
    }

    public Pose2d selectClosestCoralStation() {
        return r.drive
                .getPose()
                .nearest(
                        List.of(
                                Locations.getLeftGatherStation(),
                                Locations.getRightGatherStation()));
    }
}
