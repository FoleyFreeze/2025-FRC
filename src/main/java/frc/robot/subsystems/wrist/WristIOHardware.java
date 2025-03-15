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
import frc.robot.util.PhoenixUtil;

public class WristIOHardware implements WristIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absEncoder;
    private SparkClosedLoopController closedLoopController;
    WristCals k;

    public WristIOHardware(WristCals k) {
        this.k = k;
        motor = new SparkMax(15, MotorType.kBrushless);

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(4, 0.006, 1).outputRange(-0.4, 0.4).iZone(0.05);
        config.closedLoopRampRate(0);

        config.smartCurrentLimit(30);
        config.secondaryCurrentLimit(60);

        config.encoder.positionConversionFactor(1.0 / k.gearRatio);

        config.absoluteEncoder.zeroCentered(true);
        // -0.1689 is from skipping 3? teeth
        config.absoluteEncoder.zeroOffset(
                1 - 0.2155 - 0.1689 - 0.0956 - 0.1132 - 0.1044 + 0.0995 + 0.0605 + 0.3229 - 0.117
                        + 0.1580 + 0.0546);
        config.absoluteEncoder.positionConversionFactor(1);

        PhoenixUtil.tryUntilOkRev(
                5,
                () ->
                        motor.configure(
                                config,
                                ResetMode.kResetSafeParameters,
                                PersistMode.kPersistParameters));

        absEncoder = motor.getAbsoluteEncoder();
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();

        zero();
    }

    @Override
    public void updateInputs(WristIOInputs inputs) {
        double relEncPos = encoder.getPosition();
        double rawVolts = motor.getBusVoltage();

        inputs.wristPositionRad = Units.rotationsToRadians(relEncPos);
        inputs.wristVelocityRadPerSec =
                Units.rotationsPerMinuteToRadiansPerSecond(encoder.getVelocity());
        inputs.wristAppliedVolts = rawVolts * motor.getAppliedOutput();
        inputs.wristCurrent = motor.getOutputCurrent();
        inputs.wristTempF = motor.getMotorTemperature() * 9 / 5.0 + 32;

        inputs.absEncAngleRaw = absEncoder.getPosition();
        inputs.absEncAngleRel =
                Units.rotationsToRadians(convertAbsToRel(inputs.absEncAngleRaw, relEncPos));

        inputs.wristConnected = rawVolts > 6;
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
        // 0.4515abs == 0 deg rel
        encoder.setPosition(absEncVal / k.gearRatioToAbsEncoder - Units.degreesToRotations(69.5));
        // encoder.setPosition(convertAbsToRel(absEncVal, encoder.getPosition()));
    }

    @Override
    public void resetPositionTo(double degrees) {
        encoder.setPosition(Units.degreesToRotations(degrees));
    }

    @Override
    public void setWristVolts(double volts) {
        motor.setVoltage(volts);
    }

    // if the relative and abs encoders are way apart, this resets the rel to "true" zero
    @Override
    public void superZero() {
        encoder.setPosition(0);
        zero();
    }

    // convert absenc value to relenc value
    public double convertAbsToRel(double absEnc, double relEnc) {
        absEnc =
                (absEnc - k.absEncOffset + (k.startEncVal / (360.0 / k.gearRatioToAbsEncoder))) % 1;

        if (absEnc > 0.5) {
            absEnc = absEnc - 1;
        } else if (absEnc < -0.5) {
            absEnc = absEnc + 1;
        }

        absEnc = absEnc / k.gearRatioToAbsEncoder;
        double maxZeroArea = absEnc + 1.0 / k.gearRatioToAbsEncoder / 2.0;
        double extraRevOfAbsEnc = Math.ceil((relEnc - maxZeroArea) * k.gearRatioToAbsEncoder);

        if (extraRevOfAbsEnc < Math.ceil(k.startEncVal / (360.0 / k.gearRatioToAbsEncoder))) {
            return absEnc;
        } else {
            double result = extraRevOfAbsEnc / k.gearRatioToAbsEncoder + absEnc;
            return result;
        }
    }
}
