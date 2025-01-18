package frc.robot.subsystems.elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOHardware implements ElevatorIO {

  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> elevatorAppliedVolts;
  private final StatusSignal<Current> elevatorCurrent;

  private final TalonFX elevatorTalon;

  public ElevatorIOHardware() {
    elevatorTalon = new TalonFX(0, "*");

    elevatorPosition = elevatorTalon.getPosition();
    elevatorVelocity = elevatorTalon.getVelocity();
    elevatorAppliedVolts = elevatorTalon.getMotorVoltage();
    elevatorCurrent = elevatorTalon.getStatorCurrent();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var elevatorStatus =
        BaseStatusSignal.refreshAll(
            elevatorPosition, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);

    // inputs.elevatorConnected = elevatorConnectedDebounce.calculate(elevatorStatus.isOK());
    inputs.elevatorPosition = elevatorPosition.getValue();
    inputs.elevatorVelocity = elevatorVelocity.getValue();
    inputs.elevatorAppliedVolts = elevatorAppliedVolts.getValue();
    inputs.elevatorCurrent = elevatorCurrent.getValue();
  }

  @Override
  public void setElevatorVelocity(AngularVelocity velocityRadPerSec) {}

  @Override
  public void setElevatorVolts(Voltage volts) {}

  @Override
  public void setElevatorPosition(Angle motorPosition) {}
}
