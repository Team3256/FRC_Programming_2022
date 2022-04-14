package frc.robot.helper.LED.helpers;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LEDStrip {
    AddressableLED led;
    AddressableLEDBuffer ledBuffer;
    public LEDStrip(int leds, int channel){
        led = new AddressableLED(channel);
        ledBuffer = new AddressableLEDBuffer(leds);
    }
    public AddressableLEDBuffer getLedBuffer(){
        return ledBuffer;
    }
    public void set(){
        led.setData(ledBuffer);
    }
}
