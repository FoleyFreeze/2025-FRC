package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.RobotContainer;
import org.littletonrobotics.junction.Logger;

public class BotState {

    private RobotContainer r;

    public boolean hasCoral = true;
    public boolean hasAlgae = false;
    public boolean hasStop = false;
    public boolean pathComplete = true; // default true when no path is running
    public boolean onTarget = false;
    public boolean inLocalPosePhase = false;

    public Trigger hasCoralT = new Trigger(() -> hasCoral);
    public Trigger hasAlgaeT = new Trigger(() -> hasAlgae);
    public Trigger hasStopT = new Trigger(() -> hasStop);
    public Trigger pathCompleteT = new Trigger(() -> pathComplete);
    public Trigger onTargetT =
            new Trigger(
                    () ->
                            onTarget
                                    && (r.flysky.topRightSWD.getAsBoolean()
                                            || DriverStation.isAutonomous()));
    public Trigger inLocalPosePhaseT = new Trigger(() -> inLocalPosePhase);

    public BotState(RobotContainer r) {
        this.r = r;
    }

    public void setCoral() {
        hasCoral = true;
        hasAlgae = false;
    }

    public void setAlgae() {
        hasAlgae = true;
        hasCoral = false;
    }

    public void clear() {
        hasAlgae = false;
        hasCoral = false;
    }

    public void periodic() {
        // the transition out is in DriveCommands.driveTo
        if (r.startLocalPosePhase.getAsBoolean()) {
            inLocalPosePhase = true;
        }

        Logger.recordOutput("State/hasCoral", hasCoral);
        Logger.recordOutput("State/hasAlgae", hasAlgae);
        Logger.recordOutput("State/hasStop", hasStop);
        Logger.recordOutput("State/pathComplete", pathComplete);
        Logger.recordOutput("State/onTarget", onTarget);
        Logger.recordOutput("State/inSlowDrivePhase", r.inSlowDrivePhase);
        Logger.recordOutput("State/inLocalPosePhase", inLocalPosePhase);
    }
}
