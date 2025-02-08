package frc.robot.commands;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;

public enum SuperstructureLocation {
    HOLD(4, 0, 0),
    LEVEL1(12, 30, 0),
    LEVEL2(6.1, 34, 99),
    LEVEL3(17.3, 24.3, 68),
    LEVEL4(41.7, 37, 85),

    PRENET(0, 0, 0),
    NET(0, 0, 0),

    ALGAE_LEVEL12(0, 0, 0),
    ALGAE_LEVEL23(0, 0, 0),

    PRE_LEVEL1(0, 0, 0),
    PRE_LEVEL2(0, 0, 0),
    PRE_LEVEL3(0, 0, 0),
    PRE_LEVEL4(50, 60, 0),

    FLOOR_GATHER_ALGAE(0, 0, 0),
    SCORE_PROCESSOR(0, 0, 0),
    INTAKE(3, -32, -59),
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
