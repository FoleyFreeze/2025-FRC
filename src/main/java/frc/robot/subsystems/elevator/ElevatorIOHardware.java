package frc.robot.subsystems.elevator;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Radians;
import static edu.wpi.first.units.Units.RadiansPerSecond;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.VelocityTorqueCurrentFOC;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.StaticFeedforwardSignValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.util.Units;
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

    private final boolean useTorqueControl = true;

    ElevatorCals k;

    // velocity control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);
    private final PositionVoltage positionVoltageRequest = new PositionVoltage(0.0);
    private final VelocityVoltage velocityVoltageRequest = new VelocityVoltage(0.0);

    // torque control requests
    // private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);
    private final MotionMagicTorqueCurrentFOC positionTorqueCurrentRequest =
            new MotionMagicTorqueCurrentFOC(0);
    private final VelocityTorqueCurrentFOC velocityTorqueCurrentRequest =
            new VelocityTorqueCurrentFOC(0.0);

    private final Debouncer elevatorConnectedDebounce = new Debouncer(0.5);

    public ElevatorIOHardware(ElevatorCals k) {
        this.k = k;
        elevatorTalon = new TalonFX(2, "*");

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        config.CurrentLimits.StatorCurrentLimitEnable = true;
        config.CurrentLimits.StatorCurrentLimit = 120;
        config.CurrentLimits.SupplyCurrentLimitEnable = false;
        config.CurrentLimits.SupplyCurrentLimit = 80;
        config.CurrentLimits.SupplyCurrentLowerLimit = 40;
        config.CurrentLimits.SupplyCurrentLowerTime = 1;

        config.Feedback.SensorToMechanismRatio = k.gearRatio;

        // config.MotionMagic

        config.Slot0.GravityType = GravityTypeValue.Elevator_Static;
        config.Slot0.StaticFeedforwardSign = StaticFeedforwardSignValue.UseVelocitySign;
        config.Slot0.kG = 0;
        config.Slot0.kS = 5;
        config.Slot0.kV = 2;
        config.Slot0.kA = 2;
        config.Slot0.kP = 250;
        config.Slot0.kI = 400; // 15;
        config.Slot0.kD = 15;

        config.TorqueCurrent.PeakForwardTorqueCurrent = 140;
        config.TorqueCurrent.PeakReverseTorqueCurrent = -100;

        config.MotionMagic.MotionMagicCruiseVelocity = 15;
        config.MotionMagic.MotionMagicAcceleration = 35;

        config.Voltage.PeakForwardVoltage = 6;
        config.Voltage.PeakReverseVoltage = -3;

        tryUntilOk(5, () -> elevatorTalon.getConfigurator().apply(config, 0.25));
        zero();

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
        inputs.elevatorPositionInches =
                elevatorPosition.getValue().in(Radians) * k.drumRadiusInches;
        inputs.elevatorVelocityInchesPerSec =
                elevatorVelocity.getValue().in(RadiansPerSecond) * k.drumRadiusInches;
        inputs.elevatorAppliedVolts = elevatorAppliedVolts.getValue().in(Volts);
        inputs.elevatorCurrentAmps = elevatorCurrent.getValue().in(Amps);
        inputs.elevatorTempFahrenheit = elevatorTemp.getValue().in(Fahrenheit);
    }

    @Override
    public void setElevatorVelocity(double velocityInchesPerSec) {
        double velocity = Units.radiansToRotations(velocityInchesPerSec / k.drumRadiusInches);
        if (useTorqueControl) {
            elevatorTalon.setControl(
                    velocityTorqueCurrentRequest.withVelocity(velocity).withAcceleration(0));
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
        double motorPosition = Units.radiansToRotations(motorPositionInches / k.drumRadiusInches);
        if (useTorqueControl) {
            elevatorTalon.setControl(positionTorqueCurrentRequest.withPosition(motorPosition));
        } else {
            elevatorTalon.setControl(positionVoltageRequest.withPosition(motorPosition));
        }
    }

    @Override
    public void zero() {
        tryUntilOk(5, () -> elevatorTalon.setPosition(0.0, 0.25));
    }

    @Override
    public void setBrake(boolean on) {
        if (on) {
            elevatorTalon.setNeutralMode(NeutralModeValue.Brake);
        } else {
            elevatorTalon.setNeutralMode(NeutralModeValue.Coast);
        }
    }
}
