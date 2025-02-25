package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public enum SuperstructureLocation {
    // elevator, arm, wrist
    // inches, degress, degrees
    HOLD(0, 0, -50),
    HOLD_GATHER(0, 0, -75),
    VERT_ALGAE(19, 0, 193),
    HOLD_ALGAE(14.7, 90, 228),

    LEVEL1(12, 30, 0),
    LEVEL2(4.2 - 0.5, 12, 75),
    LEVEL3(17.5 - 0.5, 12, 75), // LEVEL3(17.5, 14, 88 + 3),
    LEVEL4(40.7 + 1, 20, 86),

    PRENET(0, 0, 0),
    NET(0, 0, 0),

    ALGAE_LEVEL_2_3(14.7, 90, 228),
    ALGAE_LEVEL_3_4(28, 90, 228),

    PRE_LEVEL1(0, 0, 0),
    PRE_LEVEL2(0, 0, 0),
    PRE_LEVEL3(0, 0, 0),
    PRE_LEVEL4(50, 60, 0),

    FLOOR_GATHER_ALGAE(0, 0, 0),
    SCORE_PROCESSOR(4, 90, 248),
    INTAKE(0, -48.5 + .5, -67.46 + 2), // was 0, 48.5 + .5, -67.46
    SAFE(0, 0, 0);

    public final Distance eleHeight;
    public final Angle armAngle;
    public final Angle wristAngle;

    private SuperstructureLocation(double height, double arm, double wrist) {
        eleHeight = Inches.of(height);
        armAngle = Degrees.of(arm);
        wristAngle = Degrees.of(wrist);
    }
}
