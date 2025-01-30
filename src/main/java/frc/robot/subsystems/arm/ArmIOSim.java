package frc.robot.subsystems.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import org.littletonrobotics.junction.Logger;

public class ArmIOSim implements ArmIO {
  // TODO: all numbers are zero at the moment, also the notationed overrides break everything for
  // some reason
  private final SingleJointedArmSim sim;
  static final double pi2 = Math.PI / 2.0;

  ArmCals k;

  double inputVoltage;

  boolean isClosedLoop = false;
  double targetAngle;

  ArmFeedforward ffController = new ArmFeedforward(3.6, Units.lbsToKilograms(8), 1);
  ProfiledPIDController positionPID =
      new ProfiledPIDController(
          20,
          0,
          0, // volts/rad
          new TrapezoidProfile.Constraints(2, 4)); // rads

  public ArmIOSim(ArmCals k) {
    this.k = k;
    double inertiaDist = Units.inchesToMeters(k.armLength);
    double inertia = Units.lbsToKilograms(8) * inertiaDist * inertiaDist;

    sim =
        new SingleJointedArmSim(
            DCMotor.getNEO(1),
            22.29,
            inertia,
            Units.inchesToMeters(k.armLength),
            0,
            Math.PI,
            true,
            pi2,
            0.001,
            0);
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    if (isClosedLoop) {
      // pid to target angle
      inputVoltage = positionPID.calculate(sim.getAngleRads());
      inputVoltage +=
          ffController.calculate(
              positionPID.getSetpoint().position, positionPID.getSetpoint().velocity);
      Logger.recordOutput("Arm/Target", positionPID.getGoal().position);
      Logger.recordOutput("Arm/CommandPos", positionPID.getSetpoint().position);
      Logger.recordOutput("Arm/CommandVel", positionPID.getSetpoint().velocity);
      inputVoltage = MathUtil.clamp(inputVoltage, -12, 12);
      sim.setInputVoltage(inputVoltage);
    }

    sim.update(0.02);

    inputs.armPositionRad = sim.getAngleRads() - pi2;
    inputs.armVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.armAppliedVolts = inputVoltage;
    inputs.armCurrent = sim.getCurrentDrawAmps();
    inputs.armTempF = 0;
  }

  @Override
  public void setArmVolts(double inputVoltage) {
    this.inputVoltage = MathUtil.clamp(inputVoltage, 0, 0);
    isClosedLoop = false;
    sim.setInputVoltage(this.inputVoltage);
  }

  @Override
  public void setArmPosition(double positionRad) {
    // adjust position
    positionRad += pi2;
    isClosedLoop = true;
    positionPID.setGoal(positionRad);
    targetAngle = positionRad;
  }

  @Override
  public void setArmVelocity(double velocityRadPerSec) {}
}
