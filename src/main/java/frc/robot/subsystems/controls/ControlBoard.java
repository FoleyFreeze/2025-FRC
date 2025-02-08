package frc.robot.subsystems.controls;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.commands.SuperstructureLocation;

public class ControlBoard {

    public enum ReefSticks {
        A,
        B,
        C,
        D,
        E,
        F,
        G,
        H,
        I,
        J,
        K,
        L;
    }

    public ReefSticks lastPressed;

    public Rotation2d getAlignAngle() {
        switch (lastPressed) {
            case A:
            case B:
                return Rotation2d.fromDegrees(0);
            case C:
            case D:
                return Rotation2d.fromDegrees(60);
            case E:
            case F:
                return Rotation2d.fromDegrees(120);
            case G:
            case H:
                return Rotation2d.fromDegrees(180);
            case I:
            case J:
                return Rotation2d.fromDegrees(240);
            case K:
            case L:
                return Rotation2d.fromDegrees(300);
            default:
                return Rotation2d.fromDegrees(0);
        }
    }

    public SuperstructureLocation getLevelFromController(CommandXboxController controller) {
        int level = 0;
        if (controller.button(8).getAsBoolean()) {
            level = 1;
        } else if (controller.button(9).getAsBoolean()) {
            level = 3;
        } else {
            level = 2;
        }

        return getLevelLocation(level);
    }

    public SuperstructureLocation getLevelLocation(int reefLevel) {
        switch (reefLevel) {
            case 1:
                return SuperstructureLocation.LEVEL1;
            case 2:
                return SuperstructureLocation.LEVEL2;
            case 3:
                return SuperstructureLocation.LEVEL3;
            case 4:
                return SuperstructureLocation.LEVEL4;
            default:
                return SuperstructureLocation.INTAKE;
        }
    }
}
