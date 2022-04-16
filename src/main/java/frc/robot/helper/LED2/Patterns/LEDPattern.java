package frc.robot.helper.LED2.Patterns;

import frc.robot.helper.LED2.Color;

public class LEDPattern {
    public int startPercentage;
    public int endPercentage;
    public int length;
    public Color[] totalPattern;

    public LEDPattern(int start, int end, Color[] totalPattern){
        startPercentage=start;
        endPercentage=end;
        length=end-start+1;
        this.totalPattern=totalPattern;
    }

    public void update(){
        for (int i=0;i<length;i++){
            totalPattern[startPercentage+i].set(0,0,0);
        }
    }
}
