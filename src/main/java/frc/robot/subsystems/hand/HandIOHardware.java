package frc.robot.subsystems.hand;

import static edu.wpi.first.units.Units.Amps;
import static edu.wpi.first.units.Units.Fahrenheit;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.util.PhoenixUtil.tryUntilOk;

import au.grapplerobotics.ConfigurationFailedException;
import au.grapplerobotics.LaserCan;
import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Current;
import edu.wpi.first.units.measure.Temperature;
import edu.wpi.first.units.measure.Voltage;

public class HandIOHardware implements HandIO {
    private final StatusSignal<Angle> handPosition;
    private final StatusSignal<AngularVelocity> handVelocity;
    private final StatusSignal<Voltage> handAppliedVolts;
    private final StatusSignal<Current> handCurrent;
    private final StatusSignal<Temperature> handTemp;
    HandIOInputs inputs;

    private final TalonFX handTalon;

    HandCals k;

    LaserCan laserCan;

    // velocity control requests
    private final VoltageOut voltageRequest = new VoltageOut(0);

    // torque control requests
    // private final TorqueCurrentFOC torqueCurrentRequest = new TorqueCurrentFOC(0);

    private final Debouncer handConnectedDebounce = new Debouncer(0.5);

    CurrentLimitsConfigs currLimConfig = new CurrentLimitsConfigs();

    public HandIOHardware(HandCals k) {
        this.k = k;
        handTalon = new TalonFX(17, "rio");

        TalonFXConfiguration config = new TalonFXConfiguration();
        config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
        config.MotorOutput.NeutralMode = NeutralModeValue.Brake;

        currLimConfig.StatorCurrentLimitEnable = true;
        currLimConfig.StatorCurrentLimit = 40;
        currLimConfig.SupplyCurrentLimitEnable = false;
        currLimConfig.SupplyCurrentLimit = 80;
        currLimConfig.SupplyCurrentLowerLimit = 40;
        currLimConfig.SupplyCurrentLowerTime = 1;
        config.withCurrentLimits(currLimConfig);

        config.Feedback.SensorToMechanismRatio = 1;

        tryUntilOk(5, () -> handTalon.getConfigurator().apply(config, 0.25));

        handPosition = handTalon.getPosition();
        handVelocity = handTalon.getVelocity();
        handAppliedVolts = handTalon.getMotorVoltage();
        handCurrent = handTalon.getStatorCurrent();
        handTemp = handTalon.getDeviceTemp();

        laserCan = new LaserCan(21);

        try {
            laserCan.setRangingMode(LaserCan.RangingMode.SHORT);
            laserCan.setRegionOfInterest(new LaserCan.RegionOfInterest(0, 0, 6, 6));
            laserCan.setTimingBudget(LaserCan.TimingBudget.TIMING_BUDGET_33MS);
        } catch (ConfigurationFailedException e) {
            System.out.println("LaserCan Configuration failed! " + e);
        }
    }

    @Override
    public void updateInputs(HandIOInputs inputs) {
        var handStatus =
                BaseStatusSignal.refreshAll(
                        handPosition, handVelocity, handAppliedVolts, handCurrent, handTemp);
        inputs.handConnected = handConnectedDebounce.calculate(handStatus.isOK());
        inputs.handAppliedVolts = handAppliedVolts.getValue().in(Volts);
        inputs.handCurrent = handCurrent.getValue().in(Amps);
        inputs.handTempF = handTemp.getValue().in(Fahrenheit);

        LaserCan.Measurement measurement = laserCan.getMeasurement();
        inputs.laserStatus = measurement.status;
        inputs.laserDistmm = measurement.distance_mm;
    }

    @Override
    public void setHandVolts(double volts) {
        handTalon.setControl(voltageRequest.withOutput(volts));
    }

    @Override
    public void changeCurrentLimit(double newLimit) {
        if (newLimit != currLimConfig.StatorCurrentLimit) {
            currLimConfig.StatorCurrentLimit = newLimit;
            // note this blocks for 100ms, thats probably fine?
            StatusCode code = handTalon.getConfigurator().apply(currLimConfig);
            if (code != StatusCode.OK) {
                System.out.println(
                        "Failed to set Hand CurrentLimits !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            }
        }
    }
}
