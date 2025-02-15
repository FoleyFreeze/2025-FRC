package frc.robot.subsystems.arm;

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

public class ArmIOHardware implements ArmIO {
    private final SparkMax motor;
    private final RelativeEncoder encoder;
    private final AbsoluteEncoder absEncoder;
    private SparkClosedLoopController closedLoopController;

    ArmCals k;

    public ArmIOHardware(ArmCals cals) {
        k = cals;
        motor = new SparkMax(16, MotorType.kBrushless);
        absEncoder = motor.getAbsoluteEncoder();
        encoder = motor.getEncoder();
        closedLoopController = motor.getClosedLoopController();

        SparkMaxConfig config = new SparkMaxConfig();
        config.closedLoop.pid(2.5, 0, 0).outputRange(-0.3, 0.3);
        config.closedLoopRampRate(0);

        config.smartCurrentLimit(30);
        config.secondaryCurrentLimit(60);

        config.encoder.positionConversionFactor(1.0 / cals.gearRatio);
        config.absoluteEncoder.positionConversionFactor(1.0 / cals.gearRatioToAbsEncoder);

        motor.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        zero();
    }

    @Override
    public void updateInputs(ArmIOInputs inputs) {
        double relEnc = encoder.getPosition();

        inputs.armPositionRad = Units.rotationsToRadians(relEnc);
        inputs.armVelocityRadPerSec =
                Units.radiansPerSecondToRotationsPerMinute(encoder.getVelocity());
        inputs.armAppliedVolts = motor.getBusVoltage() * motor.getAppliedOutput();
        inputs.armCurrent = motor.getOutputCurrent();
        inputs.armTempF = motor.getMotorTemperature() * 9 / 5.0 + 32;

        inputs.absEncAngleRaw = absEncoder.getPosition();
        
    }

    @Override
    public void setArmPosition(double motorPositionRad) {
        double rotations = Units.radiansToRotations(motorPositionRad);
        closedLoopController.setReference(rotations, ControlType.kPosition);
    }

    @Override
    public void setArmVolts(double volts) {
        motor.setVoltage(volts);
    }

    @Override
    public void zero() {
        // read the absolute encoder and reset the relative one
        double absEncVal = absEncoder.getPosition();
        encoder.setPosition(absEncVal);
        absEncVal = (1 - (absEncVal * k.gearRatioToAbsEncoder)) / k.gearRatioToAbsEncoder;
        encoder.setPosition(0 - Units.degreesToRotations(83));
    }

    public double convertAbsToRel(double absEnc, double relEnc) {
        absEnc = absEnc - k.armEncOffsetDegres;
        if (absEnc > 0.5) {
            absEnc = absEnc - 1;
        } else if (absEnc < 0.5) {
            absEnc = absEnc - 1;
        }

        absEnc = absEnc / k.gearRatio;
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
