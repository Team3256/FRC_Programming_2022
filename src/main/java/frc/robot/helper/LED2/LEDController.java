package frc.robot.helper.LED2;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.helper.LED2.Patterns.BallColorPattern;
import frc.robot.helper.LED2.Patterns.LEDPattern;

import java.util.ArrayList;

public class LEDController {
    ArrayList<LEDContainer> ledContainers = new ArrayList<>();
    ArrayList<LEDPattern> ledPatterns = new ArrayList<>();
    Color[] totalPattern = new Color[100];
    AddressableLED led;
    AddressableLEDBuffer buffer;

    public LEDController(){
        led = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(75);
        led.setLength(buffer.getLength());

        ledContainers.add(new LEDContainer(0, 24));
        ledContainers.add(new LEDContainer(25, 49));
        ledContainers.add(new LEDContainer(50, 74));

        for (int i=0;i<100;i++){
            totalPattern[i] = new Color(0,0,0);
        }

        ledPatterns.add(new BallColorPattern( 0,49,totalPattern));
        ledPatterns.add(new BallColorPattern(50,99,totalPattern));

        led.setData(buffer);
        led.start();
    }

    public void periodic(){
        for (LEDPattern pattern : ledPatterns){
            pattern.update();
        }
        for (LEDContainer container : ledContainers){
            container.display(totalPattern, buffer);
        }
        led.setData(buffer);
    }
}
