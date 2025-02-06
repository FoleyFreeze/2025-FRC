package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.subsystems.wrist.WristIO.WristIOInputs;
import org.littletonrobotics.junction.Logger;

public class WristIOSim implements WristIO {

    private final SingleJointedArmSim sim;
    static final double pi2 = Math.PI / 2.0;

    WristCals k;

    double inputVoltage;

    boolean isClosedLoop = false;
    double targetAngle;

    ArmFeedforward ffController = new ArmFeedforward(3.6, Units.lbsToKilograms(5), 1);
    ProfiledPIDController positionPID =
            new ProfiledPIDController(
                    20,
                    0,
                    0, // volts/rad
                    new TrapezoidProfile.Constraints(2, 4)); // rads

    public WristIOSim(WristCals k) {
        this.k = k;
        double inertiaDist = Units.inchesToMeters(k.wristLength);
        double inertia = Units.lbsToKilograms(5) * inertiaDist * inertiaDist;

        sim =
                new SingleJointedArmSim(
                        DCMotor.getNEO(1),
                        24.3,
                        inertia,
                        Units.inchesToMeters(k.wristLength),
                        -pi2,
                        pi2,
                        true,
                        0,
                        0.001,
                        0);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        if (isClosedLoop) {
            // pid to target angle
            inputVoltage = positionPID.calculate(sim.getAngleRads());
            inputVoltage +=
                    ffController.calculate(
                            positionPID.getSetpoint().position, positionPID.getSetpoint().velocity);
            Logger.recordOutput("Wrist/Target", positionPID.getGoal().position);
            Logger.recordOutput("Wrist/CommandPos", positionPID.getSetpoint().position);
            Logger.recordOutput("Wrist/CommandVel", positionPID.getSetpoint().velocity);
            inputVoltage = MathUtil.clamp(inputVoltage, -12, 12);
            sim.setInputVoltage(inputVoltage);
        }

        sim.update(0.02);

        inputs.wristPositionRad = sim.getAngleRads();
        inputs.wristVelocityRadPerSec = sim.getVelocityRadPerSec();
        inputs.wristAppliedVolts = inputVoltage;
        inputs.wristCurrent = sim.getCurrentDrawAmps();
        inputs.wristTempF = 0;
    }

    @Override
    public void setWristPosition(double positionRad) {
        isClosedLoop = true;
        positionPID.setGoal(positionRad);
        targetAngle = positionRad;
    }

    @Override
    public void setWristVolts(double inputVoltage) {
        this.inputVoltage = MathUtil.clamp(inputVoltage, -12, 12);
        isClosedLoop = false;
        sim.setInputVoltage(this.inputVoltage);
    }
}
