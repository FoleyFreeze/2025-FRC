package frc.robot.subsystems.elevator;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public class ElevatorIOSim implements ElevatorIO {

  private final ElevatorSim sim;

  ElevatorCals k;

  double inputVoltage;

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
    sim.update(0.02);

    inputs.elevatorConnected = true;
    inputs.elevatorPositionInches = Units.metersToInches(sim.getPositionMeters());
    inputs.elevatorVelocityInchesPerSec = Units.metersToInches(sim.getVelocityMetersPerSecond());
    inputs.elevatorAppliedVolts = inputVoltage;
    inputs.elevatorCurrentAmps = sim.getCurrentDrawAmps();
    inputs.elevatorTempFahrenheit = 0;
  }

  @Override
  public void setElevatorVolts(double inputVoltage) {
    this.inputVoltage = MathUtil.clamp(inputVoltage, -12, 12);
    sim.setInputVoltage(this.inputVoltage);
  }

  @Override
  public void setElevatorPosition(double motorPositionInches) {}

  @Override
  public void setElevatorVelocity(double velocityInchesPerSec) {}
}
