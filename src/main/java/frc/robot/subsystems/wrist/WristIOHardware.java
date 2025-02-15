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
    WristCals k;

    public WristIOHardware(WristCals k) {
        this.k = k;
        motor = new SparkMax(15, MotorType.kBrushless);
        absEncoder = motor.getAbsoluteEncoder();
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(2.5, 0, 0).outputRange(-0.3, 0.3);
        config.closedLoopRampRate(0);

        config.smartCurrentLimit(30);
        config.secondaryCurrentLimit(60);

        config.encoder.positionConversionFactor(1.0 / k.gearRatio);
        config.absoluteEncoder.positionConversionFactor(1);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        zero();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        inputs.wristPositionRad = Units.rotationsToRadians(encoder.getPosition());
        inputs.wristVelocityRadPerSec =
                Units.radiansPerSecondToRotationsPerMinute(encoder.getVelocity());
        inputs.wristAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.wristCurrent = motor.getOutputCurrent();
        inputs.wristTempF = motor.getMotorTemperature() * 9 / 5.0 + 32;
        inputs.absEncAngleRad = Units.rotationsToRadians(absEncoder.getPosition());
    }

    @Override
    public void setWristPosition(double motorPosition) {
        closedLoopController.setReference(
                Units.radiansToRotations(motorPosition), ControlType.kPosition);
    }

    @Override
    public void zero() {
        // read the absolute encoder and reset the relative one
        double absEncVal = absEncoder.getPosition();
        encoder.setPosition(0 - 0.25); // add 0.25 revolutions to start at 90deg
    }

    // convert absenc value to relenc value
    public double convertAbsToRel(double absEnc, double relEnc) {
        absEnc = absEnc - k.absEncOffsetDeg;

        if (absEnc > 0.5) {
            absEnc = absEnc - 1;
        } else if (absEnc < 0.5) {
            absEnc = absEnc + 1;
        }

        absEnc = absEnc / k.gearRatioToAbsEncoder;
        double maxZeroArea = absEnc + 1 / k.gearRatioToAbsEncoder / 2;
        double extraRevOfAbsEnc = Math.ceil((relEnc - maxZeroArea) * k.gearRatioToAbsEncoder);

        if (extraRevOfAbsEnc < 0) {
            return absEnc;
        } else {
            double result = extraRevOfAbsEnc / k.gearRatioToAbsEncoder + absEnc;
            return result;
        }
    }
}
