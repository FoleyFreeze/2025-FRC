package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.controls.ControlBoard.ReefSticks;
import frc.robot.util.Locations;
import frc.robot.util.SettableLoggableChooser;

public class AutonSelection {

    public RobotContainer r;
    public Command autonCommand = new InstantCommand();

    private String lastAutonString = "";
    private String lastPresetString = "";

    public enum GatherType {
        NONE,
        LEFT_FAR,
        LEFT_CLOSE,
        RIGHT_FAR,
        RIGHT_CLOSE,
        REEF,
        ZONE1,
        ZONE2,
        ZONE3,
    }

    private SettableLoggableChooser<Integer> startLocation;

    private SettableLoggableChooser<ReefSticks> scoreLoc1;
    private SettableLoggableChooser<Integer> scoreLevel1;

    private SettableLoggableChooser<GatherType> gatherChoose1;
    private SettableLoggableChooser<ReefSticks> scoreLoc2;
    private SettableLoggableChooser<Integer> scoreLevel2;

    private SettableLoggableChooser<GatherType> gatherChoose2;
    private SettableLoggableChooser<ReefSticks> scoreLoc3;
    private SettableLoggableChooser<Integer> scoreLevel3;

    private SettableLoggableChooser<GatherType> gatherChoose3;
    private SettableLoggableChooser<ReefSticks> scoreLoc4;
    private SettableLoggableChooser<Integer> scoreLevel4;

    private SettableLoggableChooser<String> presets;

    public AutonSelection(RobotContainer r) {
        this.r = r;

        ShuffleboardTab tab = Shuffleboard.getTab("Auton");

        startLocation = new SettableLoggableChooser<>("StartLoc", tab, 0, 0);
        startLocation.addDefaultOption("Left", 0);
        startLocation.addOption("Mid", 1);
        startLocation.addOption("Right", 2);

        scoreLoc1 = new SettableLoggableChooser<>("Score1Loc", tab, 0, 1);
        fillScoreLoc(scoreLoc1);
        scoreLevel1 = new SettableLoggableChooser<>("Score1Level", tab, 0, 2);
        fillScoreLevel(scoreLevel1);

        gatherChoose1 = new SettableLoggableChooser<>("Gather1", tab, 1, 0);
        FillGatherLoc(gatherChoose1);
        scoreLoc2 = new SettableLoggableChooser<>("Score2Loc", tab, 1, 1);
        fillScoreLoc(scoreLoc2);
        scoreLevel2 = new SettableLoggableChooser<>("Score2Level", tab, 1, 2);
        fillScoreLevel(scoreLevel2);

        gatherChoose2 = new SettableLoggableChooser<>("Gather2", tab, 2, 0);
        FillGatherLoc(gatherChoose2);
        scoreLoc3 = new SettableLoggableChooser<>("Score3Loc", tab, 2, 1);
        fillScoreLoc(scoreLoc3);
        scoreLevel3 = new SettableLoggableChooser<>("Score3Level", tab, 2, 2);
        fillScoreLevel(scoreLevel3);

        gatherChoose3 = new SettableLoggableChooser<>("Gather3", tab, 3, 0);
        FillGatherLoc(gatherChoose3);
        scoreLoc4 = new SettableLoggableChooser<>("Score4Loc", tab, 3, 1);
        fillScoreLoc(scoreLoc4);
        scoreLevel4 = new SettableLoggableChooser<>("Score4Level", tab, 3, 2);
        fillScoreLevel(scoreLevel4);

        presets = new SettableLoggableChooser<>("Presets", tab, 2, 4);
        presets.addDefaultOption("Score1Gather1", "Left,H,3,None,None,1,None,None,1,None,None,1");
        presets.addOption("Score2Gather1", "Mid,K,3,Left_Far,None,1,None,None,1,None,None,1");
    }

    public void periodic() {
        if (!presets.get().equals(lastPresetString)) {
            lastPresetString = presets.get();
            updateToPreset(lastPresetString);
            System.out.println("UpdatePresets");
        }

        // read string
        // compare string
        // if not same, create auton
        String s = readString();
        if (!lastAutonString.equals(s)) {
            autonCommand = buildAuton();
            lastAutonString = s;
            System.out.println("Generated new auton: " + s);
        }
    }

    public Command getAutonCommand() {
        // quick hack to run old autos
        // TODO: make cleaner
        Command oldAuto = r.getAutonomousCommand();
        if (oldAuto == null) {
            return autonCommand;
        } else {
            return oldAuto;
        }
    }

    public void updateToPreset(String in) {
        String[] parts = in.split(",");
        startLocation.setDefault(parts[0]);
        scoreLoc1.setDefault(parts[1]);
        scoreLevel1.setDefault(parts[2]);
    }

    public String readString() {
        String s = "";
        s += startLocation.getKey();
        s += scoreLoc1.getKey();
        s += scoreLevel1.getKey();
        s += gatherChoose1.getKey();
        s += scoreLoc2.getKey();
        s += scoreLevel2.getKey();
        s += gatherChoose2.getKey();
        s += scoreLoc3.getKey();
        s += scoreLevel3.getKey();
        s += gatherChoose3.getKey();
        s += scoreLoc4.getKey();
        s += scoreLevel4.getKey();
        return s;
    }

    public Command buildAuton() {
        SequentialCommandGroup c = new SequentialCommandGroup();
        c.setName("Auton");

        // start location

        Pose2d startLoc = Locations.getStartLoc(startLocation.get());
        if (startLoc == null) {
            return new InstantCommand();
        }
        r.drive.setPose(startLoc);
        // dont need as we will get a better position from the april tags
        // c.addCommands(new InstantCommand(() -> r.drive.setPose(startLoc)));

        // score 1
        ReefSticks stick = scoreLoc1.get();
        Integer level = scoreLevel1.get();
        if (stick == ReefSticks.NONE || level == null) {
            return new InstantCommand();
        }
        c.addCommands(AutonCommands.scoreCoral(scoreLoc1.get(), scoreLevel1.get()));

        // gather 1
        Pose2d gatherLoc = getGatherLocation(gatherChoose1.get());
        if (gatherLoc == null) {
            return c; // we done
        }
        c.addCommands(AutonCommands.coralStationGather(gatherLoc));

        // score 2
        stick = scoreLoc2.get();
        level = scoreLevel2.get();
        if (stick == ReefSticks.NONE || level == null) {
            return c;
        }
        c.addCommands(AutonCommands.scoreCoral(scoreLoc2.get(), scoreLevel2.get()));

        // gather 2
        gatherLoc = getGatherLocation(gatherChoose2.get());
        if (gatherLoc == null) {
            return c; // we done
        }
        c.addCommands(AutonCommands.coralStationGather(gatherLoc));

        // score 3
        stick = scoreLoc3.get();
        level = scoreLevel3.get();
        if (stick == ReefSticks.NONE || level == null) {
            return c;
        }
        c.addCommands(AutonCommands.scoreCoral(scoreLoc3.get(), scoreLevel3.get()));

        // gather 3
        gatherLoc = getGatherLocation(gatherChoose3.get());
        if (gatherLoc == null) {
            return c; // we done
        }
        c.addCommands(AutonCommands.coralStationGather(gatherLoc));

        // score 4
        stick = scoreLoc4.get();
        level = scoreLevel4.get();
        if (stick == ReefSticks.NONE || level == null) {
            return c;
        }
        c.addCommands(AutonCommands.scoreCoral(scoreLoc4.get(), scoreLevel4.get()));

        return c;
    }

    private void fillScoreLoc(SettableLoggableChooser<ReefSticks> in) {
        in.addDefaultOption("None", ReefSticks.NONE);
        in.addOption("A", ReefSticks.A);
        in.addOption("B", ReefSticks.B);
        in.addOption("C", ReefSticks.C);
        in.addOption("D", ReefSticks.D);
        in.addOption("E", ReefSticks.E);
        in.addOption("F", ReefSticks.F);
        in.addOption("G", ReefSticks.G);
        in.addOption("H", ReefSticks.H);
        in.addOption("I", ReefSticks.I);
        in.addOption("J", ReefSticks.J);
        in.addOption("K", ReefSticks.K);
        in.addOption("L", ReefSticks.L);
    }

    private void fillScoreLevel(SettableLoggableChooser<Integer> in) {
        in.addDefaultOption("1", 1);
        in.addOption("2", 2);
        in.addOption("3", 3);
        in.addOption("4", 4);
    }

    private void FillGatherLoc(SettableLoggableChooser<GatherType> in) {
        in.addDefaultOption("None", GatherType.NONE);
        in.addOption("Left_Far", GatherType.LEFT_FAR);
        in.addOption("Left_Close", GatherType.LEFT_CLOSE);
        in.addOption("Right_Far", GatherType.RIGHT_FAR);
        in.addOption("Right_Close", GatherType.RIGHT_CLOSE);
        in.addOption("Reef", GatherType.REEF);
        in.addOption("Zone1", GatherType.ZONE1);
        in.addOption("Zone2", GatherType.ZONE2);
        in.addOption("Zone3", GatherType.ZONE3);
    }

    private Pose2d getGatherLocation(GatherType gathertype) {
        Pose2d gatherLoc;
        switch (gathertype) {
            case LEFT_CLOSE:
                gatherLoc = Locations.getLeftGatherStation();
                break;
            case LEFT_FAR:
                gatherLoc = Locations.getLeftGatherStation();
                break;
            case RIGHT_CLOSE:
                gatherLoc = Locations.getRightGatherStation();
                break;
            case RIGHT_FAR:
                gatherLoc = Locations.getRightGatherStation();
                break;
            case REEF: // TODO: this
            default:
                gatherLoc = null; // we done
        }
        return gatherLoc;
    }
}
