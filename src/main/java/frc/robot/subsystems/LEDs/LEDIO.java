package frc.robot.subsystems.LEDs;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public interface LEDIO {

    public default void setData(AddressableLEDBuffer buffer) {}

    public default void reInit(AddressableLEDBuffer buffer) {}
}
