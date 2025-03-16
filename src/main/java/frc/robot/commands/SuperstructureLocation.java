package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public enum SuperstructureLocation {
    // elevator, arm, wrist
    // inches, degress, degrees
    HOLD(0.25, 5, -25),
    HOLD_GATHER(0.25, 5, -70),
    HOLD_ALGAE_XFER(16, 5, 140),

    VERT_ALGAE(19, 8, 193),
    HOLD_ALGAE(14.7, 90, 228),

    LEVEL1(0.25, 66, 15),
    LEVEL2(5.25, 14, 73),
    LEVEL3(18.5, 14, 73),
    LEVEL4(40.7 + 1.75, 20, 88),
    ALGAE_DESCORE2_3(28.5, 115, 42),
    ALGAE_DESCORE3_4(41.4, 115, 42),

    PRENET(22, 90, 228),
    NET(32, 22, 165),

    ALGAE_LEVEL_2_3(20.3 - 2, 114, 228), // cal'd for 1188
    ALGAE_LEVEL_3_4(34.9 - 2, 114, 228), // cal'd for 1188

    PRE_LEVEL1(0, 0, 0),
    PRE_LEVEL2(0, 0, 0),
    PRE_LEVEL3(0, 0, 0),
    PRE_LEVEL4(50, 60, 0),

    FLOOR_GATHER_ALGAE(0, 0, 0),
    SCORE_PROCESSOR(4, 110, 248),

    PRE_INTAKE(0.10, -49, -69.5),
    INTAKE(2.1, -56, -63),
    POST_INTAKE(0.10, -27, -30),

    CLIMB_SAFE(0.25, 45, -20),
    LOW_CLIMB(0.25, -73, -50);

    public final Distance eleHeight;
    public final Angle armAngle;
    public final Angle wristAngle;

    private SuperstructureLocation(double height, double arm, double wrist) {
        eleHeight = Inches.of(height);
        armAngle = Degrees.of(arm);
        wristAngle = Degrees.of(wrist);
    }
}
