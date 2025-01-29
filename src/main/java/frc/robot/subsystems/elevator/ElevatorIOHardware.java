package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.controls.PositionTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class ElevatorIOHardware implements ElevatorIO {

  private final StatusSignal<Angle> elevatorPosition;
  private final StatusSignal<AngularVelocity> elevatorVelocity;
  private final StatusSignal<Voltage> elevatorAppliedVolts;
  private final StatusSignal<Current> elevatorCurrent;
  private final StatusSignal<Temperature> elevatorTemp;

  private final TalonFX elevatorTalon;

  private final boolean useTorqueControl = false;

  ElevatorCals k;

  // velocity control requests
  private final VoltageOut voltageRequest = new VoltageOut(0);
  private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
  private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

  // torque control requests
  // private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
  private final PositionTorqueCurrentFOC positionTorqueCurrentRequest =
      new PositionTorqueCurrentFOC(0);
  private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
      new VelocityTorqueCurrentFOC(0.0);

  private final Debouncer elevatorConnectedDebounce = new Debouncer(0.5);

  public ElevatorIOHardware(ElevatorCals k) {
    this.k = k;
    elevatorTalon = new TalonFX(0, "*");

    elevatorPosition = elevatorTalon.getPosition();
    elevatorVelocity = elevatorTalon.getVelocity();
    elevatorAppliedVolts = elevatorTalon.getMotorVoltage();
    elevatorCurrent = elevatorTalon.getStatorCurrent();
    elevatorTemp = elevatorTalon.getDeviceTemp();
  }

  @Override
  public void updateInputs(ElevatorIOInputs inputs) {
    var elevatorStatus =
        BaseStatusSignal.refreshAll(
            elevatorPosition, elevatorVelocity, elevatorAppliedVolts, elevatorCurrent);

    inputs.elevatorConnected = elevatorConnectedDebounce.calculate(elevatorStatus.isOK());
    inputs.elevatorPositionInches = elevatorPosition.getValue().in(Radians) * k.drumRadiusInches;
    inputs.elevatorVelocityInchesPerSec =
        elevatorVelocity.getValue().in(RadiansPerSecond) * k.drumRadiusInches;
    inputs.elevatorAppliedVolts = elevatorAppliedVolts.getValue().in(Volts);
    inputs.elevatorCurrentAmps = elevatorCurrent.getValue().in(Amps);
    inputs.elevatorTempFahrenheit = elevatorTemp.getValue().in(Fahrenheit);
  }

  @Override
  public void setElevatorVelocity(double velocityInchesPerSec) {
    double velocity = velocityInchesPerSec / k.drumRadiusInches;
    if (useTorqueControl) {
      elevatorTalon.setControl(
          velocityTorqueCurrentRequest.withVelocity(velocity).withAcceleration(null));
    } else {
      elevatorTalon.setControl(velocityVoltageRequest.withVelocity(velocity));
    }
  }

  @Override
  public void setElevatorVolts(double volts) {
    elevatorTalon.setControl(voltageRequest.withOutput(volts));
  }

  @Override
  public void setElevatorPosition(double motorPositionInches) {
    double motorPosition = motorPositionInches / k.drumRadiusInches;
    if (useTorqueControl) {
      elevatorTalon.setControl(positionTorqueCurrentRequest.withPosition(motorPosition));
    } else {
      elevatorTalon.setControl(positionVoltageRequest.withPosition(motorPosition));
    }
  }
}
