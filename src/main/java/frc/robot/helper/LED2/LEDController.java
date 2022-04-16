package frc.robot.helper.LED2;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

import java.util.ArrayList;

public class LEDController {
    ArrayList<LEDContainer> ledContainers = new ArrayList<>();
    ArrayList<LEDPattern> patterns = new ArrayList<>();
    AddressableLED led;
    AddressableLEDBuffer buffer;

    public LEDController(){
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(60);

        ledContainers.add(new LEDContainer());
        ledContainers.add(new LEDContainer());
        ledContainers.add(new LEDContainer());
    }

    public void periodic(){
        for (LEDContainer container : ledContainers){
            //container.display(patterns)
        }
    }
}
