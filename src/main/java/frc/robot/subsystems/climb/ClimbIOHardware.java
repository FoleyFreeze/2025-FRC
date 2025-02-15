package frc.robot.subsystems.climb;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;

public class ClimbIOHardware implements ClimbIO {
    ClimbCals k;
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private SparkClosedLoopController closedLoopController;

    public ClimbIOHardware(ClimbCals cals) {
        this.k = k;
        motor = new SparkMax(14, MotorType.kBrushless);
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();
        // TODO: fill out the nums, yo
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(0, 0, 0).outputRange(0, 0);
        config.closedLoopRampRate(0);

        config.smartCurrentLimit(60);
        config.secondaryCurrentLimit(60);
        config.encoder.positionConversionFactor(1.0 / cals.gearRatio);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        zero();
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {
        double rawVoltage = motor.getBusVoltage();

        inputs.climbPositionRads = Units.rotationsToRadians(encoder.getPosition());
        inputs.climbVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.climbAppliedVolts = rawVoltage * motor.getAppliedOutput();
        inputs.climbCurrentAmps = motor.getOutputCurrent();
        inputs.climbTempFahrenheit = motor.getMotorTemperature() * 9 / 5.0 + 32;

        inputs.climbConnected = rawVoltage > 6;
    }

    @Override
    public void zero() {
        encoder.setPosition(0);
    }
}
