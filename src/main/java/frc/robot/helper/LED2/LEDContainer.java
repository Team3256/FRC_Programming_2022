package frc.robot.helper.LED2;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.helper.LED2.Patterns.LEDPattern;

import java.util.ArrayList;
import java.util.logging.Logger;

/**
 * class that properly displays a pattern percentage array
 * onto the specified section of the whole LED strip
 */
public class LEDContainer {
    int start;
    int end;
    int length;
    
    public LEDContainer(int start, int end){
        this.start=start;
        this.end=end;
        length=end-start+1;
    }

    public void display(Color[] totalPattern, AddressableLEDBuffer buffer) {
        //loop through every percent
        for (int i=0;i<100;i++){
            //convert percentage to pixel
            int pixel = start+(length*i/100);
            //finesse out of bound errors
            if (pixel < 0 || pixel >= buffer.getLength()) continue;
            //set the buffer pixel to the color
            Color color = totalPattern[i];
            buffer.setRGB(pixel, color.R, color.G, color.B);
        }
    }
}
