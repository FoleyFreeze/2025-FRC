package frc.robot.subsystems.arm;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class ArmIOHardware implements ArmIO {
  private final SparkMax motor;
  private final AbsoluteEncoder absEncoder;

  public ArmIOHardware() {
    motor = new SparkMax(0, MotorType.kBrushless);
    absEncoder = motor.getAbsoluteEncoder();
  }
}
