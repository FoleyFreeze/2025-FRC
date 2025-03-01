package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public enum SuperstructureLocation {
    // elevator, arm, wrist
    // inches, degress, degrees
    HOLD(0.25, 8, -50),
    HOLD_GATHER(0.25, 8, -70),
    VERT_ALGAE(19, 8, 193),
    HOLD_ALGAE(14.7, 90, 228),

    LEVEL1(0.25, 66, 15),
    LEVEL2(4.25, 14, 73),
    LEVEL3(18.5, 14, 73),
    LEVEL4(40.7 + 1.75, 20, 88),

    PRENET(0, 0, 0),
    NET(0, 0, 0),

    ALGAE_LEVEL_2_3(20.3, 114, 228), // 14.7
    ALGAE_LEVEL_3_4(34.9, 114, 228),

    PRE_LEVEL1(0, 0, 0),
    PRE_LEVEL2(0, 0, 0),
    PRE_LEVEL3(0, 0, 0),
    PRE_LEVEL4(50, 60, 0),

    FLOOR_GATHER_ALGAE(0, 0, 0),
    SCORE_PROCESSOR(4, 110, 248),
    INTAKE(0.25, -48.5 + .5, -67.5 + 2), // was 0, 48.5 + .5, -67.46
    CLIMB_SAFE(0.25, 45, -20);

    public final Distance eleHeight;
    public final Angle armAngle;
    public final Angle wristAngle;

    private SuperstructureLocation(double height, double arm, double wrist) {
        eleHeight = Inches.of(height);
        armAngle = Degrees.of(arm);
        wristAngle = Degrees.of(wrist);
    }
}
