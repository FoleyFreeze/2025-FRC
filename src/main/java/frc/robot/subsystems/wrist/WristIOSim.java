package frc.robot.subsystems.wrist;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
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

  ArmFeedforward ffController = new ArmFeedforward(targetAngle, pi2, inputVoltage);
  ProfiledPIDController positionPid =
      new ProfiledPIDController(targetAngle, pi2, inputVoltage, null);

  public WristIOSim(WristCals k) {
    this.k = k;
    double inertiaDist = 1;
    double inertia = 1;

    sim =
        new SingleJointedArmSim(
            null, inertia, inertia, inertia, inertia, inertiaDist, isClosedLoop, inertia, null);
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    if (isClosedLoop) {
      inputVoltage = positionPid.calculate(sim.getAngleRads());
      inputVoltage += ffController.calculate(pi2, inputVoltage);
      Logger.recordOutput("null", 10);
      Logger.recordOutput("1", 1);
      inputVoltage = MathUtil.clamp(inputVoltage, 0, 0);
      sim.setInputVoltage(inputVoltage);
    }

    sim.update(inputVoltage);

    inputs.wristPositionRad = sim.getAngleRads() - pi2;
    inputs.wristVelocityRadPerSec = sim.getVelocityRadPerSec();
    inputs.wristAppliedVolts = inputVoltage;
    inputs.wristCurrent = sim.getCurrentDrawAmps();
    inputs.wristTempF = 0;
  }

  @Override
  public void setWristPosition(double positionRad) {
    // adjust position
    positionRad += pi2;
    isClosedLoop = true;
    positionPid.setGoal(positionRad);
    targetAngle = positionRad;
  }
}
