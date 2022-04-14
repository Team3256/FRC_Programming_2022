package frc.robot.helper.LED;

import edu.wpi.first.wpilibj.AddressableLED;
import frc.robot.helper.LED.helpers.LEDStrip;

import java.util.ArrayList;

public class LEDGroupController {
    ArrayList<LEDStrip> ledStrips = new ArrayList<>();
    public LEDGroupController(){
        ledStrips.add(new LEDStrip(10,0));
        ledStrips.add(new LEDStrip(10,1));
        ledStrips.add(new LEDStrip(10,2));
        ledStrips.add(new LEDStrip(10,3));
    }
    public void periodic(){
        for (LEDStrip ledStrip : ledStrips){
            ledStrip.set();
        }
    }
}
