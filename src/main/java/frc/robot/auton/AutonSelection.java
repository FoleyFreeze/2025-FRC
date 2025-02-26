package frc.robot.auton;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.controls.ControlBoard.ReefSticks;
import frc.robot.util.Locations;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonSelection {

    public RobotContainer r;
    public Command autonCommand = new InstantCommand();

    private String lastAutonString = "";

    public AutonSelection(RobotContainer r) {
        this.r = r;
    }

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

    private LoggedDashboardChooser<Integer> startLocation;

    private LoggedDashboardChooser<ReefSticks> scoreLoc1;
    private LoggedDashboardChooser<Integer> scoreLevel1;

    private LoggedDashboardChooser<GatherType> gatherChoose1;
    private LoggedDashboardChooser<ReefSticks> scoreLoc2;
    private LoggedDashboardChooser<Integer> scoreLevel2;

    private LoggedDashboardChooser<GatherType> gatherChoose2;
    private LoggedDashboardChooser<ReefSticks> scoreLoc3;
    private LoggedDashboardChooser<Integer> scoreLevel3;

    private LoggedDashboardChooser<GatherType> gatherChoose3;
    private LoggedDashboardChooser<ReefSticks> scoreLoc4;
    private LoggedDashboardChooser<Integer> scoreLevel4;

    public AutonSelection() {
        startLocation = new LoggedDashboardChooser<>("StartLoc");
        startLocation.addDefaultOption("One", 0);
        startLocation.addOption("Two", 1);
        startLocation.addOption("Three", 2);

        scoreLoc1 = new LoggedDashboardChooser<>("Score1Loc");
        fillScoreLoc(scoreLoc1);
        scoreLevel1 = new LoggedDashboardChooser<>("Score1Level");
        fillScoreLevel(scoreLevel1);

        gatherChoose1 = new LoggedDashboardChooser<>("Gather1");
        FillGatherLoc(gatherChoose1);
        scoreLoc2 = new LoggedDashboardChooser<>("Score2Loc");
        fillScoreLoc(scoreLoc2);
        scoreLevel2 = new LoggedDashboardChooser<>("Score2Level");
        fillScoreLevel(scoreLevel2);

        gatherChoose2 = new LoggedDashboardChooser<>("Gather2");
        FillGatherLoc(gatherChoose2);
        scoreLoc3 = new LoggedDashboardChooser<>("Score3Loc");
        fillScoreLoc(scoreLoc3);
        scoreLevel3 = new LoggedDashboardChooser<>("Score3Level");
        fillScoreLevel(scoreLevel3);

        gatherChoose3 = new LoggedDashboardChooser<>("Gather3");
        FillGatherLoc(gatherChoose3);
        scoreLoc4 = new LoggedDashboardChooser<>("Score4Loc");
        fillScoreLoc(scoreLoc4);
        scoreLevel4 = new LoggedDashboardChooser<>("Score4Level");
        fillScoreLevel(scoreLevel4);
    }

    public void periodic() {
        // read string
        // compare string
        // if not same, create auton
        String s = readString();
        if (!lastAutonString.equals(s)) {
            autonCommand = buildAuton();
            lastAutonString = s;
        }
    }

    public Command getAutonCommand() {
        return autonCommand;
    }

    public String readString() {
        String s = "";
        s += startLocation.get().toString();
        s += scoreLoc1.get().toString();
        s += scoreLevel1.get().toString();
        s += gatherChoose1.get().toString();
        s += scoreLoc2.get().toString();
        s += scoreLevel2.get().toString();
        s += gatherChoose2.get().toString();
        s += scoreLoc3.get().toString();
        s += scoreLevel3.get().toString();
        s += gatherChoose3.get().toString();
        s += scoreLoc4.get().toString();
        s += scoreLevel4.get().toString();
        return s;
    }

    public Command buildAuton() {
        SequentialCommandGroup c = new SequentialCommandGroup();

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
            return new InstantCommand();
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
            return new InstantCommand();
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
            return new InstantCommand();
        }
        c.addCommands(AutonCommands.scoreCoral(scoreLoc4.get(), scoreLevel4.get()));
        // ...
        c.setName("Auton");
        return c;
    }

    private void fillScoreLoc(LoggedDashboardChooser<ReefSticks> in) {
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

    private void fillScoreLevel(LoggedDashboardChooser<Integer> in) {
        in.addDefaultOption("None", null);
        in.addOption("2", 2);
        in.addOption("3", 3);
        in.addOption("4", 4);
    }

    private void FillGatherLoc(LoggedDashboardChooser<GatherType> in) {
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
