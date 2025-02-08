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

    public HandIOHardware(HandCals k) {
        this.k = k;
        motor = new SparkMax(17, MotorType.kBrushless);
        encoder = motor.getEncoder();

        SparkMaxConfig config = new SparkMaxConfig();

        config.smartCurrentLimit(30);
        config.secondaryCurrentLimit(60);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    @Override
    public void updateInputs(HandIOInputs inputs) {
        inputs.handAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.handCurrent = motor.getOutputCurrent();
        inputs.handTempF = motor.getMotorTemperature();
    }

    @Override
    public void setHandVolts(double volts) {
        motor.setVoltage(volts);
    }
}
