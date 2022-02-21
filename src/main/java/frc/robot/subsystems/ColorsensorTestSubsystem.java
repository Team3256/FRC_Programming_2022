package frc.robot.subsystems;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import java.util.logging.Logger;

//NOTES:
//SENSOR WORKS TO UP TO 2 INCHES FOR BALL COLOR DETECTION
//SENSOR WORKS TO UP TO 1 INCH FOR TAPE DETECTION

public class ColorsensorTestSubsystem extends SubsystemBase {

    //default
    private static final Logger logger = Logger.getLogger(ColorsensorTestSubsystem.class.getCanonicalName());
    private final I2C.Port i2cPort = I2C.Port.kOnboard;;
    private final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);;
    private final ColorMatch colorMatcher = new ColorMatch();

    //targeting
    private final Color tape = new Color(0.251413600330047,0.476727327560996,0.272224140677223);

    //averageColor tracking
    private double redSum = 0;
    private double greenSum = 0;
    private double blueSum = 0;
    private int rounds = 0;

    public ColorsensorTestSubsystem(){
        colorMatcher.addColorMatch(tape);
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
        SmartDashboard.putNumber("Red", colorSensor.getRed());
        SmartDashboard.putNumber("Green", colorSensor.getGreen());
        SmartDashboard.putNumber("Blue", colorSensor.getBlue());
        SmartDashboard.putNumber("Proximity", colorSensor.getProximity());
    }

    public void postDetectionConfidence(){
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        SmartDashboard.putNumber("Confidence", match.confidence);
    }

    public void postAverageColor(){
        redSum+=colorSensor.getRed();
        greenSum+=colorSensor.getGreen();
        blueSum+=colorSensor.getBlue();
        rounds++;
        SmartDashboard.putNumber("Red Avg", redSum/rounds);
        SmartDashboard.putNumber("Green Avg", greenSum/rounds);
        SmartDashboard.putNumber("Blue Avg", blueSum/rounds);
    }
}