package frc.robot.subsystems.wrist;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Wrist extends SubsystemBase {
  private final WristIO io;
  private final WristIOInputsAutoLogged inputs = new WristIOInputsAutoLogged();

  public Wrist(WristIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  public double getVoltage() {
    return (inputs.wristAppliedVolts);
  }
}
