package frc.robot.subsystems.hand;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.units.measure.Voltage;

public class HandIOHardware implements HandIO {
  private final SparkMax motor;

  public HandIOHardware() {
    motor = new SparkMax(0, MotorType.kBrushless);
  }

  @Override
  public void updateInputs(HandIOInputs inputs) {
    inputs.handAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
    inputs.handCurrent = motor.getOutputCurrent();
    inputs.handTempF = motor.getMotorTemperature();
  }

  @Override
  public void setHandVolts(Voltage volts) {
    motor.setVoltage(volts);
  }
}
