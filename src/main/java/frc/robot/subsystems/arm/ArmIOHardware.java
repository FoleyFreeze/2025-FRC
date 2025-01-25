package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.RPM;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.Rotations;
import static edu.wpi.first.units.Units.Volts;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
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
    // the uhhh connected thingy doesnt exist sorry :pensive:
    inputs.armPosition = Rotations.of(encoder.getPosition());
    inputs.armVelocity = RPM.of(encoder.getVelocity());
    inputs.armAppliedVolts = Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    inputs.armCurrent = Amps.of(motor.getOutputCurrent());
    inputs.armTemp = Fahrenheit.of(motor.getMotorTemperature());
    inputs.absEncAngle = Radians.of(absEncoder.getPosition());
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
