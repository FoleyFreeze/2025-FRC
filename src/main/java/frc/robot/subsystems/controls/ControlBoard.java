package frc.robot.subsystems.controls;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import frc.robot.commands.SuperstructureLocation;
import frc.robot.util.Locations;
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
    public LoggedDashboardChooser<GatherDist> stationDist;
    public LoggedDashboardChooser<CageLocation> cageLocation;
    public LoggedDashboardChooser<Boolean> enableDistSensor;

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
        cageLocation = new LoggedDashboardChooser<>("CageLoction");
        enableDistSensor = new LoggedDashboardChooser<>("DistSensor");

        enableDistSensor.addDefaultOption("Enable", true);
        enableDistSensor.addOption("Disable", false);

        cageLocation.addDefaultOption("Left", CageLocation.LEFT);
        cageLocation.addOption("Middle", CageLocation.MIDDLE);
        cageLocation.addOption("Right", CageLocation.RIGHT);

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

        stationDist.addDefaultOption("Far", GatherDist.FAR);
        stationDist.addOption("Center", GatherDist.CENTER);
        stationDist.addOption("Close", GatherDist.CLOSE);

        climbMode.addDefaultOption("Off", false);
        climbMode.addOption("On", true);

        algaeMode.addDefaultOption("Coral", false);
        algaeMode.addOption("Algae", true);

        useShuffleboard.addDefaultOption("No", false);
        useShuffleboard.addOption("Yes", true);

        // only allow temp algae when in coral mode
        algaeButtonT.onTrue(
                new InstantCommand(
                                () -> {
                                    if (!tempScoreAlgae && !selectedAlgae)
                                        tempGatherAlgae = !tempGatherAlgae;
                                })
                        .ignoringDisable(true));
        escBtn.onTrue(
                new InstantCommand(
                                () -> {
                                    tempGatherAlgae = false;
                                    tempScoreAlgae = false;
                                })
                        .ignoringDisable(true));
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

    public static enum CageLocation {
        LEFT,
        MIDDLE,
        RIGHT
    }

    public static enum GatherDist {
        CLOSE,
        CENTER,
        FAR
    }

    public static enum DestAlgae {
        NET,
        PROC,
        CLOSEST
    }

    public ReefSticks selectedReefPos = ReefSticks.A;
    public int selectedLevel = 2;
    public Station selectedStation = Station.CLOSEST;
    public GatherDist useFarStation = GatherDist.FAR;
    public boolean selectedClimbMode = false;
    public boolean selectedAlgae = false;
    public boolean shift = false;
    public boolean submerge = false;
    public boolean tempGatherAlgae = false;
    public boolean tempScoreAlgae = false;
    public CageLocation selectedCage = CageLocation.LEFT;
    public DestAlgae selectedAlgaeTarget = DestAlgae.CLOSEST;

    public boolean autonCoralGather = false;
    public boolean autonCoralScore = false;
    public boolean autonAlgaeGather = false;
    public boolean autonAlgaeScore = false;
    public int autonTag = 0;

    public boolean enableDistanceSensor = true;

    public int lastGatherStationTag = 0;

    public Trigger climbModeT = new Trigger(() -> selectedClimbMode);
    public Trigger algaeModeT = new Trigger(() -> selectedAlgae);
    public Trigger shiftT = new Trigger(() -> shift);
    public Trigger submergeT = new Trigger(() -> submerge);
    public Trigger tempGatherAlgaeT = new Trigger(() -> tempGatherAlgae);
    public Trigger tempScoreAlgaeT = new Trigger(() -> tempScoreAlgae);

    public Trigger jogA = new Trigger(() -> cb.getRawButton(7) && !useShuffleboard.get());
    public Trigger jogB = new Trigger(() -> cb.getRawButton(8) && !useShuffleboard.get());
    public Trigger jog1 = new Trigger(() -> cb.getRawButton(9) && !useShuffleboard.get());
    public Trigger jog2 = new Trigger(() -> cb.getRawButton(10) && !useShuffleboard.get());
    public Trigger gatherBtn = new Trigger(() -> cb.getRawButton(6) && !useShuffleboard.get());
    public Trigger escBtn = new Trigger(() -> cb.getRawButton(3) && !useShuffleboard.get());
    public Trigger algaeButtonT = new Trigger(() -> cb.getRawButton(5) && !useShuffleboard.get());

    public void periodic() {
        if (!useShuffleboard.get()) {
            selectedClimbMode = cb.getRawButton(1);
            submerge = cb.getRawButton(2);
            shift = !cb.getRawButton(4); // note inverted

            if (cb.getRawAxis(2) < -0.5) {
                // selectedStation = Station.LEFT;
                useFarStation = GatherDist.FAR;
            } else if (cb.getRawAxis(2) > 0.5) {
                // selectedStation = Station.RIGHT;
                useFarStation = GatherDist.CLOSE;
            } else {
                // selectedStation = Station.CLOSEST;
                useFarStation = GatherDist.CENTER;
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
                tempGatherAlgae = false;
            } else if (cb2.getRawButton(2)) {
                selectedReefPos = ReefSticks.B;
                r.leds.ledsOff();
                r.leds.ledOutputSet(20, true);
                tempGatherAlgae = false;
            } else if (cb2.getRawButton(3)) {
                selectedReefPos = ReefSticks.C;
                r.leds.ledsOff();
                r.leds.ledOutputSet(26, true);
                tempGatherAlgae = false;
            } else if (cb2.getRawButton(4)) {
                selectedReefPos = ReefSticks.D;
                r.leds.ledsOff();
                r.leds.ledOutputSet(32, true);
                tempGatherAlgae = false;
            } else if (cb2.getRawButton(5)) {
                selectedReefPos = ReefSticks.E;
                r.leds.ledsOff();
                r.leds.ledOutputSet(38, true);
                tempGatherAlgae = false;
            } else if (cb2.getRawButton(6)) {
                selectedReefPos = ReefSticks.F;
                r.leds.ledsOff();
                r.leds.ledOutputSet(44, true);
                tempGatherAlgae = false;
            } else if (cb2.getRawButton(7)) {
                selectedReefPos = ReefSticks.G;
                r.leds.ledsOff();
                r.leds.ledOutputSet(50, true);
                tempGatherAlgae = false;
            } else if (cb2.getRawButton(8)) {
                selectedReefPos = ReefSticks.H;
                r.leds.ledsOff();
                r.leds.ledOutputSet(56, true);
                tempGatherAlgae = false;
            } else if (cb2.getPOV() == 270) {
                selectedReefPos = ReefSticks.I;
                r.leds.ledsOff();
                // starting here LEDs are BGR instead of RGB
                r.leds.ledOutputSet(60, true);
                tempGatherAlgae = false;
            } else if (cb2.getPOV() == 0) {
                selectedReefPos = ReefSticks.J;
                r.leds.ledsOff();
                r.leds.ledOutputSet(66, true);
                tempGatherAlgae = false;
            } else if (cb2.getPOV() == 90) {
                selectedReefPos = ReefSticks.K;
                r.leds.ledsOff();
                r.leds.ledOutputSet(72, true);
                tempGatherAlgae = false;
            } else if (cb2.getPOV() == 180) {
                selectedReefPos = ReefSticks.L;
                r.leds.ledsOff();
                r.leds.ledOutputSet(78, true);
                tempGatherAlgae = false;
            }

            if (cb2.getRawButton(9)) {
                selectedAlgae = true;
            } else {
                selectedAlgae = false;
            }

            if (selectedAlgae) {
                switch (selectedReefPos) {
                    case A:
                    case B:
                        r.leds.ledOutputSet(14, false);
                        r.leds.ledOutputSet(20, false);
                        r.leds.ledOutputSet(13, true);
                        r.leds.ledOutputSet(19, true);
                        break;
                    case C:
                    case D:
                        r.leds.ledOutputSet(26, false);
                        r.leds.ledOutputSet(32, false);
                        r.leds.ledOutputSet(25, true);
                        r.leds.ledOutputSet(31, true);
                        break;
                    case E:
                    case F:
                        r.leds.ledOutputSet(38, false);
                        r.leds.ledOutputSet(44, false);
                        r.leds.ledOutputSet(37, true);
                        r.leds.ledOutputSet(43, true);
                        break;
                    case G:
                    case H:
                        r.leds.ledOutputSet(50, false);
                        r.leds.ledOutputSet(56, false);
                        r.leds.ledOutputSet(49, true);
                        r.leds.ledOutputSet(55, true);
                        break;
                    case I:
                    case J:
                        r.leds.ledOutputSet(60, false);
                        r.leds.ledOutputSet(66, false);
                        r.leds.ledOutputSet(61, true);
                        r.leds.ledOutputSet(67, true);
                        break;
                    case K:
                    case L:
                        r.leds.ledOutputSet(72, false);
                        r.leds.ledOutputSet(78, false);
                        r.leds.ledOutputSet(73, true);
                        r.leds.ledOutputSet(79, true);
                        break;
                    default:
                }
            } else {
                // coral mode
                if (tempGatherAlgae && selectedReefPos != ReefSticks.NONE) {
                    switch (selectedReefPos) {
                        case A:
                            r.leds.ledOutputSet(19, true);
                            break;
                        case B:
                            r.leds.ledOutputSet(13, true);
                            break;
                        case C:
                            r.leds.ledOutputSet(31, true);
                            break;
                        case D:
                            r.leds.ledOutputSet(25, true);
                            break;
                        case E:
                            r.leds.ledOutputSet(43, true);
                            break;
                        case F:
                            r.leds.ledOutputSet(37, true);
                            break;
                        case G:
                            r.leds.ledOutputSet(55, true);
                            break;
                        case H:
                            r.leds.ledOutputSet(49, true);
                            break;
                        case I:
                            r.leds.ledOutputSet(67, true);
                            break;
                        case J:
                            r.leds.ledOutputSet(61, true);
                            break;
                        case K:
                            r.leds.ledOutputSet(79, true);
                            break;
                        case L:
                            r.leds.ledOutputSet(73, true);
                            break;
                        default:
                    }
                } else {
                    r.leds.ledOutputSet(13, false);
                    r.leds.ledOutputSet(19, false);
                    r.leds.ledOutputSet(25, false);
                    r.leds.ledOutputSet(31, false);
                    r.leds.ledOutputSet(37, false);
                    r.leds.ledOutputSet(43, false);
                    r.leds.ledOutputSet(49, false);
                    r.leds.ledOutputSet(55, false);
                    r.leds.ledOutputSet(61, false);
                    r.leds.ledOutputSet(67, false);
                    r.leds.ledOutputSet(73, false);
                    r.leds.ledOutputSet(79, false);
                    switch (selectedReefPos) {
                        case A:
                            r.leds.ledOutputSet(14, true);
                            break;
                        case B:
                            r.leds.ledOutputSet(20, true);
                            break;
                        case C:
                            r.leds.ledOutputSet(26, true);
                            break;
                        case D:
                            r.leds.ledOutputSet(32, true);
                            break;
                        case E:
                            r.leds.ledOutputSet(38, true);
                            break;
                        case F:
                            r.leds.ledOutputSet(44, true);
                            break;
                        case G:
                            r.leds.ledOutputSet(50, true);
                            break;
                        case H:
                            r.leds.ledOutputSet(56, true);
                            break;
                        case I:
                            r.leds.ledOutputSet(60, true);
                            break;
                        case J:
                            r.leds.ledOutputSet(66, true);
                            break;
                        case K:
                            r.leds.ledOutputSet(72, true);
                            break;
                        case L:
                            r.leds.ledOutputSet(78, true);
                            break;
                        default:
                    }
                }
            }

        } else {
            selectedReefPos = letter.get();
            selectedLevel = level.get();
            selectedClimbMode = climbMode.get();
            selectedAlgae = algaeMode.get();
            useFarStation = stationDist.get();
        }

        enableDistanceSensor = enableDistSensor.get();

        if (r.flysky.topLeftSWBHi.getAsBoolean()) {
            selectedAlgaeTarget = DestAlgae.NET;
        } else if (r.flysky.topLeftSWBLo.getAsBoolean()) {
            selectedAlgaeTarget = DestAlgae.PROC;
        } else {
            selectedAlgaeTarget = DestAlgae.CLOSEST;
        }

        selectedStation = station.get();
        selectedCage = cageLocation.get();

        Logger.recordOutput("CB/SelectedReef", selectedReefPos);
        Logger.recordOutput("CB/SelectedLevel", selectedLevel);
        Logger.recordOutput("CB/SelectedStation", selectedStation);
        Logger.recordOutput("CB/SelectedClimb", selectedClimbMode);
        Logger.recordOutput("CB/SelectedAlgae", selectedAlgae);
        Logger.recordOutput("CB/lastGatherStationTag", lastGatherStationTag);
        Logger.recordOutput("CB/SelectedCage", selectedCage);
        Logger.recordOutput("CB/SelectedAlgaeTarget", selectedAlgaeTarget);

        Logger.recordOutput("CB/FakeAuto/Gather", autonCoralGather);
        Logger.recordOutput("CB/FakeAuto/Score", autonCoralScore);
        Logger.recordOutput("CB/FakeAuto/Tag", autonTag);

        Logger.recordOutput("CB/tempGatherAlgae", tempGatherAlgae);
        Logger.recordOutput("CB/tempScoreAlgae", tempScoreAlgae);
    }

    // left means true
    public Pose2d selectCoralStation() {
        switch (selectedStation) {
            case LEFT:
                switch (useFarStation) {
                    case FAR:
                    default:
                        return Locations.getLeftGatherStationFar();
                    case CENTER:
                        return Locations.getLeftGatherStationCenter();
                    case CLOSE:
                        return Locations.getLeftGatherStationClose();
                }
            case RIGHT:
                switch (useFarStation) {
                    case FAR:
                    default:
                        return Locations.getRightGatherStationFar();
                    case CENTER:
                        return Locations.getRightGatherStationCenter();
                    case CLOSE:
                        return Locations.getRightGatherStationClose();
                }
            case CLOSEST:
            default:
                switch (useFarStation) {
                    case FAR:
                    default:
                        return selectClosestFarCoralStation();
                    case CENTER:
                        return selectClosestCenterCoralStation();
                    case CLOSE:
                        return selectClosestCloseCoralStation();
                }
        }
    }

    public Rotation2d selectGatherAngle() {
        return selectCoralStation().getRotation().plus(Rotation2d.k180deg);
    }

    public Pose2d getPathPose() {
        if (selectedLevel == 1) {
            return Locations.getLevel1ReefLocation(selectedReefPos);
        } else {
            return Locations.getReefLocation(selectedReefPos);
        }
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
        return getAlgaeReefHeight(selectedReefPos);
    }

    public SuperstructureLocation getAlgaeReefHeight(ReefSticks stick) {
        switch (stick) {
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
        Pose2d left = Locations.getLeftGatherStationFar();
        Pose2d right = Locations.getRightGatherStationFar();

        Pose2d bot = r.drive.getGlobalPose();

        boolean leftCloser = getDistFromPoses(left, bot) < getDistFromPoses(right, bot);

        if (leftCloser) {
            lastGatherStationTag = Locations.isBlue() ? 13 : 1;
            return left;
        } else {
            lastGatherStationTag = Locations.isBlue() ? 12 : 2;
            return right;
        }
    }

    public Pose2d selectClosestCloseCoralStation() {
        Pose2d left = Locations.getLeftGatherStationClose();
        Pose2d right = Locations.getRightGatherStationClose();

        Pose2d bot = r.drive.getGlobalPose();

        boolean leftCloser = getDistFromPoses(left, bot) < getDistFromPoses(right, bot);

        if (leftCloser) {
            lastGatherStationTag = Locations.isBlue() ? 13 : 1;
            return left;
        } else {
            lastGatherStationTag = Locations.isBlue() ? 12 : 2;
            return right;
        }
    }

    public Pose2d selectClosestCenterCoralStation() {
        Pose2d left = Locations.getLeftGatherStationCenter();
        Pose2d right = Locations.getRightGatherStationCenter();

        Pose2d bot = r.drive.getGlobalPose();

        boolean leftCloser = getDistFromPoses(left, bot) < getDistFromPoses(right, bot);

        if (leftCloser) {
            lastGatherStationTag = Locations.isBlue() ? 13 : 1;
            return left;
        } else {
            lastGatherStationTag = Locations.isBlue() ? 12 : 2;
            return right;
        }
    }

    public double getDistFromPoses(Pose2d p1, Pose2d p2) {
        return p1.getTranslation().getDistance(p2.getTranslation());
    }

    public Pose2d selectApproachingStation() {
        Pose2d robotPos = r.drive.getGlobalPose();
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

    public boolean getAlgaeScoreLoc() {
        switch (selectedAlgaeTarget) {
            case NET:
                return true;
            case PROC:
                return false;
            case CLOSEST:
            default:
                // score at net if not within 9ft of proc
                return Locations.getProcLoc()
                                .getTranslation()
                                .getDistance(r.drive.getGlobalPose().getTranslation())
                        > Units.feetToMeters(9);
        }
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
