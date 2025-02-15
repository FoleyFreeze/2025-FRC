package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class WristCals {
    double wristLength = 6;
    double g1 = 13.122;
    double g2 = 30.0 / 18.0;
    double g3 = 40.0 / 36.0;
    double gearRatio = g1*g2*g3;
    public double gearRatioToAbsEncoder = g2 * g3;
    double absEncOffset = 0;

    Angle minLocalWristAngle = Degrees.of(-110);
    Angle minLocalWristAngleCoral = Degrees.of(-90);
    Angle maxLocalWristAngle = Degrees.of(80);

    double closeEnough = Units.degreesToRadians(5);
}
