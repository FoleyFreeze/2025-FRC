package frc.robot.subsystems.hand;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

public class HandIOHardware implements HandIO {
    private final SparkMax motor;
    private RelativeEncoder encoder;
    HandCals k;
    HandIOInputs inputs;

    public HandIOHardware(HandCals k) {
        this.k = k;
        motor = new SparkMax(17, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();
        config.inverted(true);
        config.smartCurrentLimit(100);
        config.secondaryCurrentLimit(124);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        zero();
    }

    @Override
    public void updateInputs(HandIOInputs inputs) {
        double rawVolts = motor.getBusVoltage();
        inputs.handConnected = rawVolts > 6;

        inputs.handAppliedVolts = rawVolts * motor.getAppliedOutput();
        inputs.handCurrent = motor.getOutputCurrent();
        inputs.handTempF = motor.getMotorTemperature() * 9 / 5.0 + 32;
    }

    @Override
    public void setHandVolts(double volts) {
        motor.setVoltage(volts);
    }

    public double getHandCurrent() {
        return inputs.handCurrent;
    }
}
