package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    public final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

    private final Alert climbDisconnectedAlert = new Alert("Climb Disconnected", AlertType.kError);

    public ClimbCals k;

    public static Climb create() {
        Climb climb;
        ClimbCals cals = new ClimbCals();
        switch (Constants.currentMode) {
            case REAL:
                climb = new Climb(new ClimbIOHardware(cals));
                // climb = new Climb(new ClimbIO() {});
                break;

            case SIM:
                climb = new Climb(new ClimbIOSim(cals));
                break;

            default:
                climb = new Climb(new ClimbIO() {});
                break;
        }

        climb.k = cals;
        return climb;
    }

    public Climb(ClimbIO io) {
        this.io = io;
    }

    @Override
    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Climb", inputs);

        climbDisconnectedAlert.set(!inputs.climbConnected);
    }

    public double getVoltage() {
        return (inputs.climbAppliedVolts);
    }

    public Command setClimbVoltage(double volts) {
        return new RunCommand(() -> setVolts(volts), this);
    }

    public void setVolts(double volts) {
        // climb height limit... lower number = higher climb
        if (volts > 0 && inputs.climbAbsPosition < 0.190) { // 0.15
            // fully in
            volts = 0;
        } else if (volts < 0 && inputs.climbAbsPosition > 0.509) {
            // fully out
            volts = 0;
        }
        io.setClimbVolts(volts);
    }

    public Command overrideClimbVoltage(double volts) {
        return new RunCommand(() -> overrideVolts(volts));
    }

    public void overrideVolts(double volts) {
        io.setClimbVolts(volts);
    }

    public void stop() {
        io.setClimbVolts(0);
    }

    public Command stopCmd() {
        return new RunCommand(this::stop, this);
    }

    public void setBrake(boolean on) {
        io.setBrake(on);
    }
}
