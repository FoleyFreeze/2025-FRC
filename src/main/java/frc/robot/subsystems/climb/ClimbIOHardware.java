package frc.robot.subsystems.climb;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.util.Units;
import frc.robot.util.PhoenixUtil;

public class ClimbIOHardware implements ClimbIO {
    ClimbCals k;
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absEnc;
    private SparkClosedLoopController closedLoopController;
    SparkMaxConfig config = new SparkMaxConfig();

    public ClimbIOHardware(ClimbCals cals) {
        this.k = cals;
        motor = new SparkMax(14, MotorType.kBrushless);

        config.inverted(true);
        config.idleMode(IdleMode.kBrake);
        config.closedLoop.pid(0, 0, 0).outputRange(0, 0);
        config.closedLoopRampRate(0);

        config.smartCurrentLimit(60);
        config.secondaryCurrentLimit(60);
        config.encoder.positionConversionFactor(1.0 / cals.gearRatio);

        PhoenixUtil.tryUntilOkRev(
                5,
                () ->
                        motor.configure(
                                config,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));

        encoder = motor.getEncoder();
        absEnc = motor.getAbsoluteEncoder();
        closedLoopController = motor.getClosedLoopController();

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
        inputs.climbAbsPosition = absEnc.getPosition();

        inputs.climbConnected = rawVoltage > 6;
    }

    @Override
    public void zero() {
        encoder.setPosition(0);
    }

    @Override
    public void setClimbVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void setBrake(boolean on) {
        config.idleMode(on ? IdleMode.kBrake : IdleMode.kCoast);
        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kNoPersistParameters);
    }
}
