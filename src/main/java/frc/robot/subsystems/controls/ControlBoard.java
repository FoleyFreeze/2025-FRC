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

    public static final boolean useCntrlBoard = false;

    public Joystick cb;
    public Joystick cb2;
    public LoggedDashboardChooser<Integer> level;
    public LoggedDashboardChooser<ReefSticks> letter;
    public LoggedDashboardChooser<Station> station;
    public LoggedDashboardChooser<Boolean> climbMode;
    public LoggedDashboardChooser<Boolean> algaeMode;
    public LoggedDashboardChooser<Boolean> useShuffleboard;

    RobotContainer r;

    public ControlBoard(RobotContainer r) {
        this.r = r;

        cb = new Joystick(1);
        cb2 = new Joystick(2);
        level = new LoggedDashboardChooser<>("Level");
        letter = new LoggedDashboardChooser<>("Letter");
        station = new LoggedDashboardChooser<>("Station");
        climbMode = new LoggedDashboardChooser<>("climbMode");
        algaeMode = new LoggedDashboardChooser<>("AlgaeMode");
        useShuffleboard = new LoggedDashboardChooser<>("UseShuffleboard");

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

        station.addDefaultOption("Closest", Station.CLOSEST);
        station.addDefaultOption("Left", Station.LEFT);
        station.addOption("Right", Station.RIGHT);

        climbMode.addDefaultOption("Off", false);
        climbMode.addOption("On", true);

        algaeMode.addDefaultOption("Coral", false);
        algaeMode.addOption("Algae", true);

        useShuffleboard.addDefaultOption("No", false);
        useShuffleboard.addOption("Yes", true);
    }

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
        L,
        NONE,
    }

    public static enum Station{
        LEFT,
        RIGHT,
        CLOSEST
    }

    public ReefSticks selectedReefPos;
    public int selectedLevel;
    public Station selectedStation;
    public boolean selectedClimbMode;
    public boolean selectedAlgae;

    public void periodic() {
        if (!useShuffleboard.get()) {
            selectedClimbMode = cb.getRawButton(1);

            if (cb.getRawAxis(2) > .5) {
                selectedStation = Station.LEFT;
            } else if (cb.getRawAxis(3) > .5) {
                selectedStation = Station.RIGHT;
            } else {
                selectedStation = Station.CLOSEST;
            }


            if (cb.getPOV() == 270) {
                selectedLevel = 1;
            } else if (cb.getPOV() == 180) {
                selectedLevel = 2;
            } else if (cb.getPOV() == 90) {
                selectedLevel = 3;
            } else if (cb.getPOV() == 0) {
                selectedLevel = 4;
            }

            if (cb2.getRawButton(1)) {
                selectedReefPos = ReefSticks.A;
            } else if (cb2.getRawButton(2)) {
                selectedReefPos = ReefSticks.B;
            } else if (cb2.getRawButton(3)) {
                selectedReefPos = ReefSticks.C;
            } else if (cb2.getRawButton(4)) {
                selectedReefPos = ReefSticks.D;
            } else if (cb2.getRawButton(5)) {
                selectedReefPos = ReefSticks.E;
            } else if (cb2.getRawButton(6)) {
                selectedReefPos = ReefSticks.F;
            } else if (cb2.getRawButton(7)) {
                selectedReefPos = ReefSticks.G;
            } else if (cb2.getRawButton(8)) {
                selectedReefPos = ReefSticks.H;
            } else if (cb2.getPOV() == 270) {
                selectedReefPos = ReefSticks.I;
            } else if (cb2.getPOV() == 0) {
                selectedReefPos = ReefSticks.J;
            } else if (cb2.getPOV() == 90) {
                selectedReefPos = ReefSticks.K;
            } else if (cb2.getPOV() == 180) {
                selectedReefPos = ReefSticks.L;
            }

            if (cb2.getRawButton(9)) {
                selectedAlgae = true;
            } else {
                selectedAlgae = false;
            }
        } else {
            selectedReefPos = letter.get();
            selectedLevel = level.get();
            selectedStation = station.get();
            selectedClimbMode = climbMode.get();
            selectedAlgae = algaeMode.get();
        }
    }

    public Trigger climbModeT = new Trigger(() -> selectedClimbMode);
    public Trigger algaeModeT = new Trigger(() -> selectedAlgae);

    // left means true
    public Pose2d selectCoralStation() {
        switch(selectedStation){
            case LEFT:
            return Locations.getLeftGatherStationFar();
            case RIGHT:
            return Locations.getRightGatherStationFar();
            case CLOSEST:
            return selectClosestCoralStation();
        }
    }

    public Rotation2d selectGatherAngle() {
        return selectCoralStation().getRotation().plus(Rotation2d.k180deg);
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

    /*
    public SuperstructureLocation getAlgaeLevelFromController(Flysky controller) {
        if (controller.botRightSWHLo.getAsBoolean()) {
            return SuperstructureLocation.ALGAE_LEVEL_2_3;
        } else if (controller.botRightSWHHi.getAsBoolean()) {
            return SuperstructureLocation.ALGAE_LEVEL_3_4;
        } else {
            return SuperstructureLocation.ALGAE_LEVEL_2_3;
        }
    }
    */

    public SuperstructureLocation getAlgaeLevel() {
        switch (selectedLevel) {
            case 1:
            case 2:
                return SuperstructureLocation.ALGAE_LEVEL_2_3;
            case 3:
            case 4:
            default:
                return SuperstructureLocation.ALGAE_LEVEL_3_4;
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
                                Locations.getLeftGatherStationFar(),
                                Locations.getRightGatherStationFar()));
    }
}
