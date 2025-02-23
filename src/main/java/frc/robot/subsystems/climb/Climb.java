package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import org.littletonrobotics.junction.Logger;

public class Climb extends SubsystemBase {
    private final ClimbIO io;
    private final ClimbIOInputsAutoLogged inputs = new ClimbIOInputsAutoLogged();

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
    }

    public double getVoltage() {
        return (inputs.climbAppliedVolts);
    }

    public Command setClimbVoltage(double volts) {
        return new RunCommand(() -> io.setClimbVolts(volts), this);
    }

    public void stop() {
        io.setClimbVolts(0);
    }

    public Command stopCmd() {
        return new RunCommand(this::stop, this);
    }
}
