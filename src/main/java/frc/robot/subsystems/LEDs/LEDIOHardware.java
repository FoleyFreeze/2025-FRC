package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDIOHardware implements LEDIO {

    AddressableLED leds = new AddressableLED(0);

    public LEDIOHardware(AddressableLEDBuffer buffer) {
        leds.setLength(buffer.getLength());
        leds.start();
    }

    @Override
    public void setData(AddressableLEDBuffer buffer) {
        leds.setData(buffer);
    }
}
