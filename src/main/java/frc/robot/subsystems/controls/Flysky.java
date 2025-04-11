package frc.robot.subsystems.controls;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Flysky {

    public Joystick flysky;

    // public Trigger leftTriggerSWE = new Trigger(() -> flysky.getRawAxis(2) > .25);
    // public Trigger rightTriggerSWG = new Trigger(() -> flysky.getRawAxis(3) > .25);
    public boolean leftTriggerBool = false;
    public boolean rightTriggerBool = false;
    public Trigger leftTriggerSWE =
            new Trigger(
                    () -> {
                        leftTriggerBool = debounceHyst(flysky.getRawAxis(2), leftTriggerBool);
                        return leftTriggerBool;
                    });
    public Trigger rightTriggerSWG =
            new Trigger(
                    () -> {
                        rightTriggerBool = debounceHyst(flysky.getRawAxis(3), rightTriggerBool);
                        return rightTriggerBool;
                    });

    public Trigger upLTRIM = new Trigger(() -> flysky.getRawButton(10));
    public Trigger downLTRIM = new Trigger(() -> flysky.getRawButton(11));
    public Trigger leftLTRIM = new Trigger(() -> flysky.getRawButton(13));
    public Trigger rightLTRIM = new Trigger(() -> flysky.getRawButton(12));

    public Trigger upRTRIM = new Trigger(() -> flysky.getRawButton(14));
    public Trigger downRTRIM = new Trigger(() -> flysky.getRawButton(15));
    public Trigger rightRTRIM = new Trigger(() -> flysky.getRawButton(16));
    public Trigger leftRTRIM = new Trigger(() -> flysky.getRawButton(17));

    // algae switch
    public Trigger topRightSWD = new Trigger(() -> flysky.getRawButton(5));
    public Trigger topLeftSWA = new Trigger(() -> flysky.getRawButton(1));
    public Trigger topLeftSWBHi = new Trigger(() -> flysky.getRawButton(2));
    public Trigger topLeftSWBLo = new Trigger(() -> flysky.getRawButton(3));
    public Trigger topRightMomentSWC = new Trigger(() -> flysky.getRawButton(4));
    public Trigger botLeftSWFHi = new Trigger(() -> flysky.getRawButton(7));
    public Trigger botLeftSWFLo = new Trigger(() -> flysky.getRawButton(6));
    public Trigger botRightSWHHi = new Trigger(() -> flysky.getRawButton(9));
    public Trigger botRightSWHLo = new Trigger(() -> flysky.getRawButton(8));
    public Trigger fillerButton = new Trigger(() -> flysky.getRawButton(0));

    public Flysky() {
        flysky = new Joystick(0);
    }

    public double getLeftX() {
        return flysky.getRawAxis(0);
    }

    public double getLeftY() {
        return flysky.getRawAxis(1);
    }

    public double getRightX() {
        return flysky.getRawAxis(4);
    }

    public double getLeftDial() {
        return (flysky.getRawAxis(5) + 1) / 2.0;
    }

    public double getRightDial() {
        return (flysky.getRawAxis(6) + 1) / 2.0;
    }

    // require > 0.5 to set and < -0.5 to unset
    public boolean debounceHyst(double in, boolean prev) {
        if (prev) {
            return in > -0.5;
        } else {
            return in > 0.5;
        }
    }
}
