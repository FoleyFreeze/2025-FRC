package frc.robot.subsystems.endeffector;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class EndEffectorIOHardware implements EndEffectorIO {
  private final SparkMax motor;
  private final AbsoluteEncoder absEncoder;

  public EndEffectorIOHardware() {
    motor = new SparkMax(0, MotorType.kBrushless);
    absEncoder = motor.getAbsoluteEncoder();
  }
}
