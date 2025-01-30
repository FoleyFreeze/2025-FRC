package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;

public class ArmIOSim implements ArmIO {
//TODO: all numbers are zero at the moment, also the notationed overrides break everything for some reason
  private final SingleJointedArmSim sim;

  ArmCals k;

  double inputVoltage;

  public ArmIOSim(ArmCals k) {
    this.k = k;
    sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(0),
            0,
            Units.lbsToKilograms(0),
            Units.inchesToMeters(k.armthingradius),
            Units.inchesToMeters(0),
            Units.inchesToMeters(0),
            true,
            0,
            0.01,
            0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    sim.update(0.02);

    inputs.armPositionRad = sim.getAngleRads();
    inputs.armVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.armAppliedVolts = inputVoltage;
    inputs.armCurrent = sim.getCurrentDrawAmps();
    inputs.armTempF = 0;
  }
//@Override
  public void setArmVolts(double inputVoltage) {
    this.inputVoltage = MathUtil.clamp(inputVoltage, 0, 0);
    sim.setInputVoltage(inputVoltage);
  }
//@Override
  public void setArmPosition(double motorVelocityRadPerSec) {}

  @Override
  public void setArmVelocity(double velocityRadPerSec) {}
}
