package frc.robot.helper.LED2.Patterns;
import frc.robot.helper.LED2.Color;

/**
 * first half of pattern displays color of first ball in the robot
 * second half of pattern displays color of second ball in the robot
 * (red or blue)
 * pattern will take up 0 to 49 percentage range
 */
public class BallColorPattern extends LEDPattern {

    public BallColorPattern(Color[] totalPattern) {
        super(0, 49, totalPattern);
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
