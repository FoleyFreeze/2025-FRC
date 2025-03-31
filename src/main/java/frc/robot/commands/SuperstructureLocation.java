package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public enum SuperstructureLocation {
    // elevator, arm, wrist
    // inches, degress, degrees
    HOLD(0.25, 5, -25 + 65),
    HOLD_GATHER(0.25, 5, -55), // 70
    HOLD_ALGAE_XFER(16, 5, 140),

    VERT_ALGAE(19, 8, 193),
    HOLD_ALGAE(14.7, 90, 228),

    // LEVEL1(0.25, 66, 15),
    LEVEL1(0.25, 65, 46),
    LEVEL2(5.25, 14, 75 + 10),
    LEVEL3(18.5, 14, 75 + 10),
    LEVEL4(40.7 + 1.75 + 0.5, 20, 92.5 + 11),
    ALGAE_DESCORE2_3(28.5 - 4.5, 115 - 22, 42),
    ALGAE_DESCORE3_4(41.4 - 3.5, 115 - 22, 42),
    ALGAE_DESCORE2_3_LOW(26.5 - 4.5, 115 - 22, 42),
    ALGAE_DESCORE3_4_LOW(39.4 - 3.5, 115 - 22, 42),

    PRENET(18, 90, 228),
    NET(34, 22, 165),

    ALGAE_LEVEL_2_3(20.3, 114, 228), // cal'd for 1188
    ALGAE_LEVEL_3_4(34.9, 114, 228), // cal'd for 1188

    PRE_LEVEL1(0, 0, 0),
    PRE_LEVEL2(0, 0, 0),
    PRE_LEVEL3(0, 0, 0),
    PRE_LEVEL4(50, 60, 0),

    FLOOR_GATHER_ALGAE(0, 0, 0),
    SCORE_PROCESSOR(1, 110, 238),

    PRE_INTAKE(-0.20, -49 + 7, -60), // -69.5
    INTAKE(1.3, -55, -60), // 2.1 -56, -63
    POST_INTAKE(-0.20, -42, -50),
    POST_INTAKE2(-0.20, -27, -10),

    CLIMB_SAFE(0.25, 45, -20),
    LOW_CLIMB(0.25, -73, -50),

    ZERO_WRIST(0.1, 9, 0);

    public final Distance eleHeight;
    public final Angle armAngle;
    public final Angle wristAngle;

    private SuperstructureLocation(double height, double arm, double wrist) {
        eleHeight = Inches.of(height);
        armAngle = Degrees.of(arm);
        wristAngle = Degrees.of(wrist);
    }
}
