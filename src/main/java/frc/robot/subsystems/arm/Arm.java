package frc.robot.subsystems.arm;

import static edu.wpi.first.units.Units.Radians;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.SuperstructureLocation;
import org.littletonrobotics.junction.Logger;

public class Arm extends SubsystemBase {
  private final ArmIO io;
  private final ArmIOInputsAutoLogged inputs = new ArmIOInputsAutoLogged();

  SuperstructureLocation target = null;

  public Arm(ArmIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
    Logger.processInputs("Arm", inputs);
  }

  public Angle getAngleRads() {
    return Radians.of(inputs.armPositionRad);
  }

  public void goTo(SuperstructureLocation loc) {
    target = loc;
    setAngle(loc.armAngle);
  }

  public void setAngle(Angle angle) {
    io.setArmPosition(angle.in(Radians));
  }
}
