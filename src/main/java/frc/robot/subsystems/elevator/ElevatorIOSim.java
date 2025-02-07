package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import org.littletonrobotics.junction.Logger;

public class ElevatorIOSim implements ElevatorIO {

    private final ElevatorSim sim;

    ElevatorCals k;

    double inputVoltage;

    boolean isClosedLoop;
    double target;

    ElevatorFeedforward ffController = new ElevatorFeedforward(0.1, 1.2, 0.5);
    ProfiledPIDController positionPID =
            new ProfiledPIDController(2, 0, 0, new TrapezoidProfile.Constraints(40, 60));

    public ElevatorIOSim(ElevatorCals k) {
        this.k = k;
        sim =
                new ElevatorSim(
                        DCMotor.getKrakenX60Foc(1),
                        k.gearRatio,
                        Units.lbsToKilograms(30),
                        Units.inchesToMeters(k.drumRadiusInches),
                        Units.inchesToMeters(0),
                        Units.inchesToMeters(53.875),
                        true,
                        0,
                        0.0001,
                        0);
    }

    @Override
    public void updateInputs(ElevatorIOInputs inputs) {
        if (isClosedLoop) {
            // run pid
            inputVoltage = positionPID.calculate(Units.metersToInches(sim.getPositionMeters()));
            inputVoltage += ffController.calculate(positionPID.getSetpoint().velocity);
            Logger.recordOutput("Elevator/Target", positionPID.getGoal().position);
            Logger.recordOutput("Elevator/CommandPos", positionPID.getSetpoint().position);
            Logger.recordOutput("Elevator/CommandVel", positionPID.getSetpoint().velocity);
            inputVoltage = MathUtil.clamp(inputVoltage, -12, 12);
            sim.setInputVoltage(inputVoltage);
        }

        sim.update(0.02);

        inputs.elevatorConnected = true;
        inputs.elevatorPositionInches = Units.metersToInches(sim.getPositionMeters());
        inputs.elevatorVelocityInchesPerSec =
                Units.metersToInches(sim.getVelocityMetersPerSecond());
        inputs.elevatorAppliedVolts = inputVoltage;
        inputs.elevatorCurrentAmps = sim.getCurrentDrawAmps();
        inputs.elevatorTempFahrenheit = 0;
    }

    @Override
    public void setElevatorVolts(double inputVoltage) {
        isClosedLoop = false;
        this.inputVoltage = MathUtil.clamp(inputVoltage, -12, 12);
        sim.setInputVoltage(this.inputVoltage);
    }

    @Override
    public void setElevatorPosition(double motorPositionInches) {
        isClosedLoop = true;
        positionPID.setGoal(motorPositionInches);
        target = motorPositionInches;
    }

    @Override
    public void setElevatorVelocity(double velocityInchesPerSec) {}
}
