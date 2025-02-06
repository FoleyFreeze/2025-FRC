package frc.robot.subsystems.hand;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

public class HandIOHardware implements HandIO {
    private final SparkMax motor;
    HandCals k;

    public HandIOHardware(HandCals k) {
        this.k = k;
        motor = new SparkMax(0, MotorType.kBrushless);
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
