package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Rotations;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Voltage;

public class ArmIOHardware implements ArmIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absEncoder;
  private SparkClosedLoopController closedLoopController;

  public ArmIOHardware() {
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
  public void setArmPosition(Angle motorPosition) {
    closedLoopController.setReference(
        motorPosition.in(Rotations), ControlType.kMAXMotionPositionControl);
  }

  @Override
  public void setArmVolts(Voltage volts) {
    motor.setVoltage(volts);
  }
}
