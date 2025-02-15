package frc.robot.subsystems.climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

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

    public double getVoltage() {
        return (inputs.climbAppliedVolts);
    }

    public Command setVoltage(double volts) {
        return new RunCommand(() -> io.setClimbVolts(volts), this);
    }
}
