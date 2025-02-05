package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.commands.SuperstructureLocation;
import org.littletonrobotics.junction.Logger;

public class Wrist extends SubsystemBase {
    private final WristIO io;
    private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

    SuperstructureLocation target = null;

    public static Wrist create() {
        Wrist wrist;
        switch (Constants.currentMode) {
            case REAL:
                wrist = new Wrist(new WristIOHardware(new WristCals()));
                break;

            case SIM:
                wrist = new Wrist(new WristIOSim(new WristCals()));
                break;

            default:
                wrist = new Wrist(new WristIO() {});
                break;
        }
        return wrist;
    }

    public Wrist(WristIO io) {
        this.io = io;
    }

    public void periodic() {
        io.updateInputs(inputs);
        Logger.processInputs("Wrist", inputs);
    }

    public double getVoltage() {
        return (inputs.wristAppliedVolts);
    }

    public void goTo(SuperstructureLocation loc) {
        target = loc;
        setAngle(loc.armAngle);
    }

    public void setAngle(Angle angle) {
        io.setWristPosition(angle.in(Radians));
    }

    public Angle getAngleRads() {
        return Radians.of(inputs.wristPositionRad);
    }
}
