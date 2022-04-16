package frc.robot.helper.LED2.Patterns;

import frc.robot.helper.LED2.Color;

public class BallColorPattern extends LEDPattern {

    public BallColorPattern(Color[] totalPattern) {
        super(0, 30, totalPattern);
    }

    @Override
    public void update(){
        for (int i=0;i<=length/2;i++){
            totalPattern[startPercentage+i].set(255,0,0);
        }
        for (int i=length/2+1;i<length;i++){
            totalPattern[startPercentage+i].set(0,0,255);
        }
    }
}
