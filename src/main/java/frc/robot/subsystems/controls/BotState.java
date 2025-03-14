package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import org.littletonrobotics.junction.Logger;

public class BotState {

    public boolean hasCoral = true;
    public boolean hasAlgae = false;
    public boolean hasStop = false;
    public boolean pathComplete = true; // default true when no path is running

    public Trigger hasCoralT = new Trigger(() -> hasCoral);
    public Trigger hasAlgaeT = new Trigger(() -> hasAlgae);
    public Trigger hasStopT = new Trigger(() -> hasStop);
    public Trigger pathCompleteT = new Trigger(() -> pathComplete);

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
        Logger.recordOutput("State/hasCoral", hasCoral);
        Logger.recordOutput("State/hasAlgae", hasAlgae);
        Logger.recordOutput("State/hasStop", hasStop);
        Logger.recordOutput("State/pathComplete", pathComplete);
    }
}
