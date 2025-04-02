package frc.robot.subsystems.hand;
// this is the grabby thing

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Hand extends SubsystemBase {
    private final HandIO io;
    private final HandIOInputsAutoLogged inputs = new HandIOInputsAutoLogged();

    private final Alert handDisconnectedAlert = new Alert("Hand Disconnected", AlertType.kError);

    public double coralGatheredDist = 170;

    public static Hand create() {
        Hand hand;
        switch (Constants.currentMode) {
            case REAL:
                hand = new Hand(new HandIOHardware(new HandCals()));
                break;

            case SIM:
                hand = new Hand(new HandIOSim());
                break;

            default:
                hand = new Hand(new HandIO() {});
                break;
        }
        return hand;
    }

    public Hand(HandIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hand", inputs);

        handDisconnectedAlert.set(!inputs.handConnected);
    }

    public double getCurrent() {
        return inputs.handCurrent;
    }

    public Command setVoltageCmd(double volts) {
        return new InstantCommand(() -> io.setHandVolts(volts), this);
    }

    public Command setCurrentLimCmd(double limit) {
        return new InstantCommand(() -> setCurrentLim(limit), this);
    }

    public void setCurrentLim(double limit) {
        io.changeCurrentLimit(limit);
    }

    public void setVoltage(double volts) {
        io.setHandVolts(volts);
    }

    public Command stop() {
        return new InstantCommand(() -> io.setHandVolts(0), this);
    }

    public Command hasCoralInBucket() {
        return new WaitUntilCommand(() -> inputs.laserDistmm < coralGatheredDist);
    }
}
