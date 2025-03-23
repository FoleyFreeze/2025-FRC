package frc.robot.subsystems.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.SuperstructureLocation;
import frc.robot.util.Locations;
import java.util.List;
import org.littletonrobotics.junction.Logger;
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
    public LoggedDashboardChooser<Boolean> stationDist;

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
        stationDist = new LoggedDashboardChooser<>("StationDist");

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
        station.addOption("Left", Station.LEFT);
        station.addOption("Right", Station.RIGHT);

        stationDist.addDefaultOption("Far", true);
        stationDist.addOption("Close", false);

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

    public static enum Station {
        LEFT,
        RIGHT,
        CLOSEST
    }

    public ReefSticks selectedReefPos = ReefSticks.A;
    public int selectedLevel = 2;
    public Station selectedStation = Station.CLOSEST;
    public boolean useFarStation = true;
    public boolean selectedClimbMode = false;
    public boolean selectedAlgae = false;
    public boolean shift = false;
    public boolean submerge = false;

    public Trigger climbModeT = new Trigger(() -> selectedClimbMode);
    public Trigger algaeModeT = new Trigger(() -> selectedAlgae);
    public Trigger shiftT = new Trigger(() -> shift);
    public Trigger submergeT = new Trigger(() -> submerge);

    public Trigger jogA = new Trigger(() -> cb.getRawButton(7) && !useShuffleboard.get());
    public Trigger jogB = new Trigger(() -> cb.getRawButton(8) && !useShuffleboard.get());
    public Trigger jog1 = new Trigger(() -> cb.getRawButton(9) && !useShuffleboard.get());
    public Trigger jog2 = new Trigger(() -> cb.getRawButton(10) && !useShuffleboard.get());
    public Trigger gatherBtn = new Trigger(() -> cb.getRawButton(6) && !useShuffleboard.get());
    public Trigger escBtn = new Trigger(() -> cb.getRawButton(3) && !useShuffleboard.get());

    public void periodic() {
        if (!useShuffleboard.get()) {
            selectedClimbMode = cb.getRawButton(1);
            submerge = cb.getRawButton(2);
            shift = !cb.getRawButton(4); // note inverted

            if (cb.getRawAxis(2) < -0.5) {
                // selectedStation = Station.LEFT;
                useFarStation = true;
            } else if (cb.getRawAxis(2) > 0.5) {
                // selectedStation = Station.RIGHT;
                useFarStation = false;
            } else {
                // selectedStation = Station.CLOSEST;
                useFarStation = true;
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
                r.leds.ledsOff();
                r.leds.ledOutputSet(14, true);
                // r.leds.localLedVal[2]++;
                // r.leds.ledValue.set(r.leds.localLedVal);
            } else if (cb2.getRawButton(2)) {
                selectedReefPos = ReefSticks.B;
                r.leds.ledsOff();
                r.leds.ledOutputSet(20, true);
                // r.leds.localLedVal[2]--;
                // r.leds.ledValue.set(r.leds.localLedVal);
                // r.leds.ledOutputSet(11, true);
            } else if (cb2.getRawButton(3)) {
                selectedReefPos = ReefSticks.C;
                r.leds.ledsOff();
                r.leds.ledOutputSet(26, true);
            } else if (cb2.getRawButton(4)) {
                selectedReefPos = ReefSticks.D;
                r.leds.ledsOff();
                r.leds.ledOutputSet(32, true);
            } else if (cb2.getRawButton(5)) {
                selectedReefPos = ReefSticks.E;
                r.leds.ledsOff();
                r.leds.ledOutputSet(38, true);
            } else if (cb2.getRawButton(6)) {
                selectedReefPos = ReefSticks.F;
                r.leds.ledsOff();
                r.leds.ledOutputSet(44, true);
            } else if (cb2.getRawButton(7)) {
                selectedReefPos = ReefSticks.G;
                r.leds.ledsOff();
                r.leds.ledOutputSet(50, true);
            } else if (cb2.getRawButton(8)) {
                selectedReefPos = ReefSticks.H;
                r.leds.ledsOff();
                r.leds.ledOutputSet(56, true);
            } else if (cb2.getPOV() == 270) {
                selectedReefPos = ReefSticks.I;
                r.leds.ledsOff();
                r.leds.ledOutputSet(60, true);
            } else if (cb2.getPOV() == 0) {
                selectedReefPos = ReefSticks.J;
                r.leds.ledsOff();
                r.leds.ledOutputSet(66, true);
            } else if (cb2.getPOV() == 90) {
                selectedReefPos = ReefSticks.K;
                r.leds.ledsOff();
                r.leds.ledOutputSet(72, true);
            } else if (cb2.getPOV() == 180) {
                selectedReefPos = ReefSticks.L;
                r.leds.ledsOff();
                r.leds.ledOutputSet(78, true);
            }

            if (cb2.getRawButton(9)) {
                selectedAlgae = true;
            } else {
                selectedAlgae = false;
            }
        } else {
            selectedReefPos = letter.get();
            selectedLevel = level.get();
            selectedClimbMode = climbMode.get();
            selectedAlgae = algaeMode.get();
            useFarStation = stationDist.get();
        }

        selectedStation = station.get();

        Logger.recordOutput("CB/SelectedReef", selectedReefPos);
        Logger.recordOutput("CB/SelectedLevel", selectedLevel);
        Logger.recordOutput("CB/SelectedStation", selectedStation);
        Logger.recordOutput("CB/SelectedClimb", selectedClimbMode);
        Logger.recordOutput("CB/SelectedAlgae", selectedAlgae);
    }

    // left means true
    public Pose2d selectCoralStation() {
        switch (selectedStation) {
            case LEFT:
                if (useFarStation) return Locations.getLeftGatherStationFar();
                else return Locations.getLeftGatherStationClose();
            case RIGHT:
                if (useFarStation) return Locations.getRightGatherStationFar();
                else return Locations.getRightGatherStationClose();
            case CLOSEST:
            default:
                if (useFarStation) return selectClosestFarCoralStation();
                else return selectClosestCloseCoralStation();
        }
    }

    public Rotation2d selectGatherAngle() {
        return selectCoralStation().getRotation().plus(Rotation2d.k180deg);
    }

    public Pose2d getPathPose() {
        return Locations.getReefLocation(selectedReefPos);
    }

    public Pose2d getAlgaePathPose() {
        return Locations.getAlgaeReefLocation(selectedReefPos);
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

    public SuperstructureLocation getAlgaeDescoreLevel() {
        switch (selectedLevel) {
            case 1:
            case 2:
                return SuperstructureLocation.ALGAE_DESCORE2_3;
            case 3:
            case 4:
            default:
                return SuperstructureLocation.ALGAE_DESCORE3_4;
        }
    }

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

    public SuperstructureLocation getAlgaeReefHeight() {
        switch (selectedReefPos) {
            case A:
            case B:
            case E:
            case F:
            case I:
            case J:
                return SuperstructureLocation.ALGAE_LEVEL_3_4;
            case C:
            case D:
            case G:
            case H:
            case K:
            case L:
            default:
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

    public Pose2d selectClosestFarCoralStation() {
        return r.drive
                .getPose()
                .nearest(
                        List.of(
                                Locations.getLeftGatherStationFar(),
                                Locations.getRightGatherStationFar()));
    }

    public Pose2d selectClosestCloseCoralStation() {
        return r.drive
                .getPose()
                .nearest(
                        List.of(
                                Locations.getLeftGatherStationClose(),
                                Locations.getRightGatherStationClose()));
    }

    public Pose2d selectApproachingStation() {
        Pose2d robotPos = r.drive.getPose();
        ChassisSpeeds robotVel = r.drive.getVelocity();

        Translation2d distL =
                Locations.getLeftGatherStationFar()
                        .getTranslation()
                        .minus(robotPos.getTranslation());
        Translation2d distR =
                Locations.getRightGatherStationFar()
                        .getTranslation()
                        .minus(robotPos.getTranslation());
        Logger.recordOutput("GatherSelect/distL", distL.getNorm());
        Logger.recordOutput("GatherSelect/distR", distR.getNorm());

        double cosDistL = dotProduct(distL, robotVel);
        double cosDistR = dotProduct(distR, robotVel);
        Logger.recordOutput("GatherSelect/cosDistL", cosDistL);
        Logger.recordOutput("GatherSelect/cosDistR", cosDistR);

        double scoreL = distL.getNorm() - cosDistL * 0.5;
        double scoreR = distR.getNorm() - cosDistR * 0.5;
        Logger.recordOutput("GatherSelect/scoreL", scoreL);
        Logger.recordOutput("GatherSelect/scoreR", scoreR);

        if (scoreR < scoreL) {
            return Locations.getRightGatherStationFar();
        } else {
            return Locations.getLeftGatherStationFar();
        }
    }

    public double dotProduct(Translation2d a, ChassisSpeeds b) {
        return a.getX() * b.vyMetersPerSecond + a.getY() * b.vxMetersPerSecond;
    }

    public SuperstructureLocation getAlgaeReefDSHeight() {
        switch (selectedReefPos) {
            case A:
            case B:
            case E:
            case F:
            case I:
            case J:
                return SuperstructureLocation.ALGAE_DESCORE3_4;
            case C:
            case D:
            case G:
            case H:
            case K:
            case L:
            default:
                return SuperstructureLocation.ALGAE_DESCORE2_3;
        }
    }

    public SuperstructureLocation getAlgaeReefDSHeightLower() {
        switch (selectedReefPos) {
            case A:
            case B:
            case E:
            case F:
            case I:
            case J:
                return SuperstructureLocation.ALGAE_DESCORE3_4_LOW;
            case C:
            case D:
            case G:
            case H:
            case K:
            case L:
            default:
                return SuperstructureLocation.ALGAE_DESCORE2_3_LOW;
        }
    }
}
