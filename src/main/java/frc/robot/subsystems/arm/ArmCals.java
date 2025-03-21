package frc.robot.subsystems.arm;

import edu.wpi.first.math.util.Units;

public class ArmCals {

    double armLength = 14.375; // this is the final name/info :)
    double gearRatio = 22.29;
    double gearRatioToAbsEncoder = 6;
    double encoderZero = 0;
    double armEncOffset = 0.5640;
    double startEncVal = -25; // -53;

    double closeEnough = Units.degreesToRadians(5); // radians
}
