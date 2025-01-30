package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;

public class ArmIOHardware implements ArmIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absEncoder;
  private SparkClosedLoopController closedLoopController;

  public ArmIOHardware(ArmCals cals) {
    motor = new SparkMax(0, MotorType.kBrushless);
    absEncoder = motor.getAbsoluteEncoder();
    encoder = motor.getEncoder();
    closedLoopController = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(ArmIOInputs inputs) {
    inputs.armPositionRad = Units.rotationsToRadians(encoder.getPosition());
    inputs.armVelocityRadPerSec = Units.radiansPerSecondToRotationsPerMinute(encoder.getVelocity());
    inputs.armAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.armCurrent = motor.getOutputCurrent();
    inputs.armTempF = motor.getMotorTemperature();
    inputs.absEncAngleRad = absEncoder.getPosition();
  }

  @Override
  public void setArmPosition(double motorPositionRad) {
    double rotations = Units.radiansToRotations(motorPositionRad);
    closedLoopController.setReference(rotations, ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setArmVolts(double volts) {
    motor.setVoltage(volts);
  }
}
