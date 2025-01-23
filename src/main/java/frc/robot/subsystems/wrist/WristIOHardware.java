package frc.robot.subsystems.wrist;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class WristIOHardware implements WristIO {
  private final SparkMax motor;
  private final AbsoluteEncoder absEncoder;

  public WristIOHardware() {
    motor = new SparkMax(0, MotorType.kBrushless);
    absEncoder = motor.getAbsoluteEncoder();
  }
}
