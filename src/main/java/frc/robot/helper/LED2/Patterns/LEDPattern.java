package frc.robot.helper.LED2.Patterns;

import frc.robot.helper.LED2.Color;

/**
 * Only extend this class, don't actually use any of these
 * The foundation of a displayable and periodically updating LEDPattern
 */
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

    //"clear" update
    public void update(){
        setRange(0,length-1,new Color(0,0,0));
    }

    public void set(int percentage, Color color){
        totalPattern[startPercentage+percentage].set(color);
    }

    public void setRange(int start, int end, Color color){
        for (int i=start;start<=end;i++){
            set(i,color);
        }
    }
}
