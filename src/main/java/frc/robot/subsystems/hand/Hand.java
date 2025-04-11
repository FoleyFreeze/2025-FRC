package frc.robot.subsystems.hand;
// this is the grabby thing

import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.SuperstructureLocation;
import org.littletonrobotics.junction.Logger;

public class Hand extends SubsystemBase {
    private final HandIO io;
    private final HandIOInputsAutoLogged inputs = new HandIOInputsAutoLogged();

    private final Alert handDisconnectedAlert = new Alert("Hand Disconnected", AlertType.kError);
    private final Alert handTempAlert = new Alert("Hand Motor Temp > 170", AlertType.kWarning);

    public double coralGatheredDist = 160;

    RobotContainer r;

    public static Hand create(RobotContainer r) {
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

        hand.r = r;
        return hand;
    }

    public Hand(HandIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Hand", inputs);

        handDisconnectedAlert.set(!inputs.handConnected);

        handTempAlert.set(inputs.handTempF > 150);
        if (inputs.handTempF > 150) {
            handTempAlert.setText(String.format("Hand Motor Temp at %.0f", inputs.handTempF));
        }
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
        return new WaitUntilCommand(
                () ->
                        inputs.laserDistmm < coralGatheredDist
                                && r.arm.atTarget(() -> SuperstructureLocation.INTAKE)
                                && r.elevator.atTarget(() -> SuperstructureLocation.INTAKE)
                                && r.wrist.atTarget(() -> SuperstructureLocation.INTAKE));
    }

    public boolean checkForCoral() {
        return inputs.laserDistmm < coralGatheredDist;
    }
}
