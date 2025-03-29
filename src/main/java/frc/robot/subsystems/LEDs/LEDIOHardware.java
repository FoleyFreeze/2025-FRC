package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOHardware implements LEDIO {

    AddressableLED leds = new AddressableLED(0);

    public LEDIOHardware(AddressableLEDBuffer buffer) {
        leds.setLength(buffer.getLength());
        // leds.setBitTiming(500, 2000, 1200, 1300); // WS2811 "slow" mode?
        // leds.setBitTiming(250, 1000, 600, 650); // WS2811 "fast" mode?
        // leds.setSyncTime(200);

        leds.start();
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        leds.setData(buffer);
        // leds.start();
    }
}
