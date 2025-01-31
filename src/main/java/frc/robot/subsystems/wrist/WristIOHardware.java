package frc.robot.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;

public class WristIOHardware implements WristIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absEncoder;
  private SparkClosedLoopController closedLoopController;

  public WristIOHardware(WristCals k) {
    motor = new SparkMax(0, MotorType.kBrushless);
    absEncoder = motor.getAbsoluteEncoder();
    encoder = motor.getEncoder();
    closedLoopController = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristPositionRad = Units.rotationsToRadians(encoder.getPosition());
    inputs.wristVelocityRadPerSec =
        Units.radiansPerSecondToRotationsPerMinute(encoder.getVelocity());
    inputs.wristAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.wristCurrent = motor.getOutputCurrent();
    inputs.wristTempF = motor.getMotorTemperature();
    inputs.absEncAngleRad = absEncoder.getPosition();
  }

  @Override
  public void setWristPosition(double motorPosition) {
    closedLoopController.setReference(
        Units.radiansToRotations(motorPosition), ControlType.kMAXMotionPositionControl);
  }
}
