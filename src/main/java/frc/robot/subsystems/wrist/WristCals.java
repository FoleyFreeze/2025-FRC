package frc.robot.subsystems.wrist;

import static edu.wpi.first.units.Units.Degrees;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;

public class WristCals {
    double wristLength = 6;
    double g1 = 10.935;
    double g2 = 30.0 / 19.0;
    double g3 = 40.0 / 30.0;
    double gearRatio = g1 * g2 * g3;
    public double gearRatioToAbsEncoder = g2 * g3;
    double absEncOffset = 0.6643;
    double startEncVal = 0; // -59;

    Angle minLocalWristAngle = Degrees.of(-124);
    Angle minLocalWristAngleCoral = Degrees.of(-90);
    Angle maxLocalWristAngleCoral = Degrees.of(80);
    Angle maxLocalWristAngle = Degrees.of(136);

    double closeEnough = Units.degreesToRadians(5);
}
