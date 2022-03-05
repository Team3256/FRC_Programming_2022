package frc.robot.helper.LED.helpers;

import frc.robot.helper.LED.PatternGenerators.PatternGenerator;

public class LEDSectionAttributes {

    public double percentageStart;
    public double percentageEnd;

    public PatternGenerator patternGenerator;

    public LEDSectionAttributes (double percentageStart, double percentageEnd, PatternGenerator patternGenerator){
        this.percentageStart = percentageStart;
        this.percentageEnd = percentageEnd;

        this.patternGenerator = patternGenerator;
    }

    public double getPercentageRange(){
        return percentageEnd - percentageStart;
    }
}
