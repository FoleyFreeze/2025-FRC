package frc.robot.subsystems.wrist;

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

public class WristIOHardware implements WristIO {
  private final SparkMax motor;
  private final RelativeEncoder encoder;
  private final AbsoluteEncoder absEncoder;
  private SparkClosedLoopController closedLoopController;

  public WristIOHardware() {
    motor = new SparkMax(0, MotorType.kBrushless);
    absEncoder = motor.getAbsoluteEncoder();
    encoder = motor.getEncoder();
    closedLoopController = motor.getClosedLoopController();
  }

  @Override
  public void updateInputs(WristIOInputs inputs) {
    inputs.wristPosition = Rotations.of(encoder.getPosition());
    inputs.wristVelocity = RPM.of(encoder.getVelocity());
    inputs.wristAppliedVolts = Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    inputs.wristCurrent = Amps.of(motor.getOutputCurrent());
    inputs.wristTemp = Fahrenheit.of(motor.getMotorTemperature());
    inputs.absEncAngle = Radians.of(absEncoder.getPosition());
  }

  @Override
  public void setWristPosition(Angle motorPosition) {
    closedLoopController.setReference(
        motorPosition.in(Rotations), ControlType.kMAXMotionPositionControl);
  }
}
