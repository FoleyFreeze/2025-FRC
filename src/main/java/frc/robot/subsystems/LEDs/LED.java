package frc.robot.subsystems.LEDs;

import static edu.wpi.first.units.Units.Percent;
import static edu.wpi.first.units.Units.Seconds;

import edu.wpi.first.networktables.BooleanPublisher;
import edu.wpi.first.networktables.IntegerArrayPublisher;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.LEDPattern;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class LED extends SubsystemBase {
    public enum LED_MODES {
        OFF(LEDPattern.solid(Color.kBlack)),
        BLUE(LEDPattern.solid(Color.kBlue)),
        GREEN(LEDPattern.solid(Color.kGreen)),
        RED(LEDPattern.solid(Color.kRed)),
        BLINK_BLUE(LEDPattern.solid(Color.kBlue).blink(Seconds.of(1), Seconds.of(1))),
        BREATHE_BLUE(
                LEDPattern.solid(Color.kBlue).breathe(Seconds.of(3))),
        WHITE(LEDPattern.solid(Color.kWhite).atBrightness(Percent.of(50))),

        RAINBOW(LEDPattern.rainbow(255, 127).scrollAtRelativeSpeed(Seconds.of(5).asFrequency()));

        public final LEDPattern pattern;

        private LED_MODES(LEDPattern pattern) {
            this.pattern = pattern;
        }
    }

    private RobotContainer r;
    public PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);

    LEDIO io;

    Timer startTimer = new Timer();
    boolean hasRestarted = false;
    AddressableLEDBuffer startBuffer = new AddressableLEDBuffer(18);
    AddressableLEDBuffer buffer = new AddressableLEDBuffer(18);

    public LED(RobotContainer r) {
        this.r = r;

        LED_MODES.WHITE.pattern.applyTo(startBuffer);

        switch (Constants.currentMode) {
            case REAL:
                io = new LEDIOHardware(startBuffer);
                break;
            case SIM:
                io = new LEDIO() {};
                break;
            case REPLAY:
            default:
                io = new LEDIO() {};
                break;
        }

        startTimer.restart();
    }

    @Override
    public void periodic() {
        ledEnable.set(true);

        if (startTimer.hasElapsed(3)) {
            if (!hasRestarted) {
                io.reInit(buffer);
                hasRestarted = true;
                System.out.println("LEDS reinit!");
            }
            io.setData(buffer);
        }

        // only underglow on the field or enabled
        if (DriverStation.isFMSAttached() || DriverStation.isEnabled()) {
            pdh.setSwitchableChannel(true);
        } else {
            pdh.setSwitchableChannel(true);
        }
    }

    public Command setLEDMode(LED_MODES mode) {
        Command c = new RunCommand(() -> mode.pattern.applyTo(buffer), this).ignoringDisable(true);
        c = c.beforeStarting(new PrintCommand("Init LED Pattern"));
        c.setName("LED Command");
        return c;
    }

    BooleanPublisher ledEnable;
    public IntegerArrayPublisher ledValue;
    public long[] localLedVal = {0, 0, 0};

    public void ledOutputSet(int value, boolean on) {
        value--;
        if (value < 32) {
            int v = 1 << value;
            if (on) {
                localLedVal[2] |= v;
            } else {
                localLedVal[2] &= ~v;
            }
        } else if (value < 64) {
            int v = 1 << (value - 32);
            if (on) {
                localLedVal[1] |= v;
            } else {
                localLedVal[1] &= ~v;
            }
        } else {
            int v = 1 << (value - 64);
            if (on) {
                localLedVal[0] |= v;
            } else {
                localLedVal[0] &= ~v;
            }
        }

        ledValue.set(localLedVal);
    }

    public void ledsOff() {
        localLedVal[0] = 0;
        localLedVal[1] = 0;
        localLedVal[2] = 0;
    }

    public void initOutputLed() {
        NetworkTableInstance nt = NetworkTableInstance.getDefault();
        NetworkTable table = nt.getTable("ControlBoard");
        ledEnable = table.getBooleanTopic("LED_Enable").publish();
        ledValue = table.getIntegerArrayTopic("LED_Output").publish();

        ledEnable.set(true);
        ledValue.set(localLedVal);
    }
}
