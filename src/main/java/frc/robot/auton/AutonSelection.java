package frc.robot.auton;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.fasterxml.jackson.annotation.JsonTypeInfo.None;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.controls.ControlBoard.ReefSticks;
import frc.robot.util.Locations;

public class AutonSelection {

    public enum GatherType{
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
    }

    private void fillScoreLoc(LoggedDashboardChooser<ReefSticks> in){
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
    private void fillScoreLevel(LoggedDashboardChooser<Integer> in){
        in.addDefaultOption("None", null);
        in.addOption("2",2);
        in.addOption("3",3);
        in.addOption("4",4);

    }

}
