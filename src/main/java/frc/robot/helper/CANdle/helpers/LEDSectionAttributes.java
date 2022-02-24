package frc.robot.helper.CANdle.helpers;

import frc.robot.helper.CANdle.PatternGenerators.PatternGenerator;

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
