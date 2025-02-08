package frc.robot.subsystems.wrist;

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

public class WristIOHardware implements WristIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absEncoder;
    private SparkClosedLoopController closedLoopController;

    public WristIOHardware(WristCals cals) {
        motor = new SparkMax(15, MotorType.kBrushless);
        absEncoder = motor.getAbsoluteEncoder();
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(2.5, 0, 0).outputRange(-0.3, 0.3);
        config.closedLoopRampRate(0);

        config.smartCurrentLimit(30);
        config.secondaryCurrentLimit(60);

        config.encoder.positionConversionFactor(1.0 / cals.gearRatioToAbsEncoder);
        config.absoluteEncoder.positionConversionFactor(1.0 / cals.gearRatioToAbsEncoder);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // read the absolute encoder and reset the relative one
        double absEncVal = absEncoder.getPosition();
        encoder.setPosition(absEncVal);
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristPositionRad = Units.rotationsToRadians(encoder.getPosition());
        inputs.wristVelocityRadPerSec =
                Units.radiansPerSecondToRotationsPerMinute(encoder.getVelocity());
        inputs.wristAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.wristCurrent = motor.getOutputCurrent();
        inputs.wristTempF = motor.getMotorTemperature();
        inputs.absEncAngleRad = absEncoder.getPosition();
    }

    @Override
    public void setWristPosition(double motorPosition) {
        closedLoopController.setReference(
                Units.radiansToRotations(motorPosition), ControlType.kMAXMotionPositionControl);
    }
}
