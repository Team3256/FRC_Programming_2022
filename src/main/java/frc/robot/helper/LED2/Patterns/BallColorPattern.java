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
        //percentage range that pattern is allocated to
        super(0, 49, totalPattern);
    }

    //event driven/state driven updates
    @Override
    public void update(){
        //set colouring of pattern
        setRange(0,24,new Color(255,0,0));
        setRange(25,49, new Color(0,0,255));
    }
}
