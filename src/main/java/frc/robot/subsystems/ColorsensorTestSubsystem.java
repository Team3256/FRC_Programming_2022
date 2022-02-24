package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.MuxedColorSensor;

import java.util.logging.Logger;

import static frc.robot.Constants.HangerConstants.TAPE_COLOR;

//NOTES:
//SENSOR WORKS TO UP TO 2 INCHES FOR BALL COLOR DETECTION
//SENSOR WORKS TO UP TO 1 INCH FOR TAPE DETECTION

public class ColorsensorTestSubsystem extends SubsystemBase {

    //default
    private static final Logger logger = Logger.getLogger(ColorsensorTestSubsystem.class.getCanonicalName());
    private final MuxedColorSensor colorSensor = MuxedColorSensor.getInstance();
    private final ColorMatch colorMatcher = new ColorMatch();

    //averageColor tracking
    private double redSum = 0;
    private double greenSum = 0;
    private double blueSum = 0;
    private int rounds = 0;

    public ColorsensorTestSubsystem(){
        colorMatcher.addColorMatch(TAPE_COLOR);
    }

    @Override
    public void periodic(){
        postSensorValues();
        postAverageColor();
        postDetectionConfidence();
    }

    //call this every time test mode is enabled
    public void resetSums(){
        rounds=0;
        redSum=0;
        greenSum=0;
        blueSum=0;
    }

    public void postSensorValues(){
        SmartDashboard.putNumber("Red", colorSensor.getLeftAlignSensorColor().red);
        SmartDashboard.putNumber("Green", colorSensor.getLeftAlignSensorColor().green);
        SmartDashboard.putNumber("Blue", colorSensor.getLeftAlignSensorColor().blue);
    }

    public void postDetectionConfidence(){
        Color detectedColor = colorSensor.getLeftAlignSensorColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        SmartDashboard.putNumber("Confidence", match.confidence);
    }

    public void postAverageColor(){
        redSum+=colorSensor.getLeftAlignSensorColor().red;
        greenSum+=colorSensor.getLeftAlignSensorColor().green;
        blueSum+=colorSensor.getLeftAlignSensorColor().blue;
        rounds++;
        SmartDashboard.putNumber("Red Avg", redSum/rounds);
        SmartDashboard.putNumber("Green Avg", greenSum/rounds);
        SmartDashboard.putNumber("Blue Avg", blueSum/rounds);
    }
}