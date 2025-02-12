package frc.robot.subsystems.climb;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkBase.ControlType;
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
    private final AbsoluteEncoder absEncoder;
    private SparkClosedLoopController closedLoopController;

    public ClimbIOHardware(ClimbCals cals) {
        this.k = k;
        motor = new SparkMax(0, MotorType.kBrushless);
        absEncoder = motor.getAbsoluteEncoder();
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();
//TODO: fill out the nums, yo
        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(0, 0, 0).outputRange(0, 0);
        config.closedLoopRampRate(0);

        config.smartCurrentLimit(0);
        config.secondaryCurrentLimit(0);
        config.encoder.positionConversionFactor(0 / cals.gearRatio);
        config.absoluteEncoder.positionConversionFactor(0 / cals.gearRatioToAbsEncoder);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        zero();
    }

    @Override
    public void updateInputs(ClimbIOInputs inputs) {}

    @Override
    public void zero() {
        double absEncVal = absEncoder.getPosition();
        encoder.setPosition(absEncVal);
    }
}
