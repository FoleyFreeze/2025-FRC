package frc.robot.subsystems.hand;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Volts;

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
    inputs.handAppliedVolts = Volts.of(motor.getBusVoltage() * motor.getAppliedOutput());
    inputs.handCurrent = Amps.of(motor.getOutputCurrent());
    inputs.handTemp = Fahrenheit.of(motor.getMotorTemperature());
  }

  @Override
  public void setHandVolts(Voltage volts) {
    motor.setVoltage(volts);
  }
}
