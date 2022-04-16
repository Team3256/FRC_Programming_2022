package frc.robot.subsystems;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LEDSubsystem extends SubsystemBase {
    private AddressableLED LED;
    private AddressableLEDBuffer LEDBuffer;

    public LEDSubsystem(){
        LED = new AddressableLED(0);
        LEDBuffer = new AddressableLEDBuffer(60);
        LED.setLength(LEDBuffer.getLength());

        LED.setData(LEDBuffer);
        LED.start();
    }

    public void on(){
        for (int i=0;i<LEDBuffer.getLength();i++){
            LEDBuffer.setRGB(i,0,0,255);
        }
        LED.setData(LEDBuffer);
    }
    int rainbowFirstPixelHue = 0;
    public void rainbow(){
        for (int i=0;i< LEDBuffer.getLength();i++){
            int hue = (rainbowFirstPixelHue + (i*180/ LEDBuffer.getLength()))%180;
            LEDBuffer.setHSV(i,hue,255,128);
        }
        rainbowFirstPixelHue += 3;
        rainbowFirstPixelHue %= 180;
        LED.setData(LEDBuffer);
    }
    public void off(){
        for (int i=0;i<LEDBuffer.getLength();i++){
            LEDBuffer.setRGB(i,0,0,0);
        }
        LED.setData(LEDBuffer);
    }
}