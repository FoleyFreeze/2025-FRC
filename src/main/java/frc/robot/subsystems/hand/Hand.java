package frc.robot.subsystems.hand;
// this is the grabby thing

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Hand extends SubsystemBase {
  private final HandIO io;
  private final HandIOInputsAutoLogged inputs = new HandIOInputsAutoLogged();

  public Hand(HandIO io) {
    this.io = io;
  }

  public void periodic() {
    io.updateInputs(inputs);
  }

  public double getVoltage() {
    return (inputs.handAppliedVolts);
  }
}
