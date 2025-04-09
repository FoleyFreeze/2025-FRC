package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLED.ColorOrder;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOHardware implements LEDIO {

    AddressableLED leds;

    public LEDIOHardware(AddressableLEDBuffer buffer) {
        leds = new AddressableLED(1);
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    @Override
    public void reInit(AddressableLEDBuffer buffer) {
        leds.close();
        leds = new AddressableLED(0);
        leds.setColorOrder(ColorOrder.kRGB);
        leds.setLength(buffer.getLength());
        leds.setData(buffer);
        leds.start();
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        leds.setData(buffer);
        // leds.start();
    }
}
