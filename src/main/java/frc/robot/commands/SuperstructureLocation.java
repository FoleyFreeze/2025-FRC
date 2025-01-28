package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public enum SuperstructureLocation {
  INTAKE(0, 0, 0),
  HOLD(0, 0, 0),
  LEVEL1(0, 0, 0),
  LEVEL2(0, 0, 0),
  LEVEL3(0, 0, 0),
  LEVEL4(0, 0, 0);

  public final Distance eleHeight;
  public final Angle armAngle;
  public final Angle wristAngle;

  private SuperstructureLocation(double height, double arm, double wrist) {
    eleHeight = Inches.of(height);
    armAngle = Degrees.of(arm);
    wristAngle = Degrees.of(wrist);
  }
}
