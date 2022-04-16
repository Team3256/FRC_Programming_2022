package frc.robot.helper.LED2;

import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import frc.robot.helper.LED2.Patterns.LEDPattern;

import java.util.ArrayList;
import java.util.logging.Logger;

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
        for (int i=0;i<100;i++){
            int pixel = start+(length*i/100);
            if (pixel < 0 || pixel >= buffer.getLength()) continue;
            Color color = totalPattern[i];
            buffer.setRGB(pixel, color.R, color.G, color.B);
        }
    }
}
