package frc.robot.helper.LEDSimple;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDSystem {
    AddressableLED led;
    AddressableLEDBuffer buffer;
    public LEDSystem() {
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(20);
    }


}
