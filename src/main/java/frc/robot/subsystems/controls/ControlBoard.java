package frc.robot.subsystems.controls;

import edu.wpi.first.math.geometry.Rotation2d;

public class ControlBoard {


    public enum ReefSticks{
        A, B, C, D, E, F, G, H, I, J, K, L;
    }

    public ReefSticks lastPressed;

    public Rotation2d getAlignAngle(){
        switch(lastPressed){
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
}
