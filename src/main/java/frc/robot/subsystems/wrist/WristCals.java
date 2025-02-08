package frc.robot.subsystems.wrist;

import edu.wpi.first.math.util.Units;

public class WristCals {
    double wristLength = 6;
    double gearRatio = 24.3;
    public double gearRatioToAbsEncoder = 30.0 / 18.0 * 40.0 / 36.0;

    double closeEnough = Units.degreesToRadians(5);
}
