package frc.robot.auton;

import frc.robot.subsystems.controls.ControlBoard.ReefSticks;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class AutonSelection {

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
        scoreLoc2 = new LoggedDashboardChooser<>("Score1Loc");
        fillScoreLoc(scoreLoc2);
        scoreLevel2 = new LoggedDashboardChooser<>("Score1Level");
        fillScoreLevel(scoreLevel2);

        gatherChoose2 = new LoggedDashboardChooser<>("Gather1");
        FillGatherLoc(gatherChoose2);
        scoreLoc3 = new LoggedDashboardChooser<>("Score1Loc");
        fillScoreLoc(scoreLoc3);
        scoreLevel3 = new LoggedDashboardChooser<>("Score1Level");
        fillScoreLevel(scoreLevel3);

        gatherChoose3 = new LoggedDashboardChooser<>("Gather1");
        FillGatherLoc(gatherChoose3);
        scoreLoc4 = new LoggedDashboardChooser<>("Score1Loc");
        fillScoreLoc(scoreLoc4);
        scoreLevel4 = new LoggedDashboardChooser<>("Score1Level");
        fillScoreLevel(scoreLevel4);
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
}
