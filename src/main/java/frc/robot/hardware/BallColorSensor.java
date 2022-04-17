package frc.robot.hardware;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.helper.BallColor;


import java.util.logging.Logger;

import static frc.robot.Constants.TransferConstants.*;

public class BallColorSensor {
    private static final Logger logger = Logger.getLogger(BallColorSensor.class.getCanonicalName());

    //Singleton Design Pattern
    private static BallColorSensor instance;
    public static BallColorSensor getInstance() {
        if (instance == null)
            instance = new BallColorSensor();
        return instance;
    }
    private final ColorSensorV3 ballColorSensor;

    private BallColorSensor(){
        ballColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
    }

    public Color getBallSensorColor() {
        return ballColorSensor.getColor();
    }

    public int getBallSensorProximity() {
        return ballColorSensor.getProximity();
    }

    private boolean colorsMatch(Color color1, Color color2, double maxConfidenceDeviation){
        ColorMatch colorMatcher = new ColorMatch();
        colorMatcher.setConfidenceThreshold(maxConfidenceDeviation);
        colorMatcher.addColorMatch(color2);
        ColorMatchResult result = colorMatcher.matchColor(color1);

        if (result == null) {
            return false;
        }
        return (1 - result.confidence) < maxConfidenceDeviation;
    }

    public BallColor ballSensorDetection(){
        Color detectedColor = getBallSensorColor();

        boolean isRedBallDetected = colorsMatch(detectedColor, RED_BALL_COLOR, MAX_BALL_COLOR_DEVIATION);
        boolean isBlueBallDetected = colorsMatch(detectedColor, BLUE_BALL_COLOR, MAX_BALL_COLOR_DEVIATION);

        if (isBlueBallDetected && isRedBallDetected) {
            logger.warning("Both RED and Blue Ball Detected! Color: ( " +
                    detectedColor.red + ", " +
                    detectedColor.green + ", " +
                    detectedColor.blue + ") \n" +
                    "Proximity: " + getBallSensorProximity());
            return BallColor.NONE;
        }

        SmartDashboard.putNumber("Ball Color RED", detectedColor.red);
        SmartDashboard.putNumber("Ball Color Green", detectedColor.green);
        SmartDashboard.putNumber("Ball Color BLUE", detectedColor.blue);
        if (isRedBallDetected) {
            return BallColor.RED;
        }
        else if (isBlueBallDetected) {
            return BallColor.BLUE;
        }
        else {
            logger.warning("NO Color detected: ( " +
                    detectedColor.red + ", " +
                    detectedColor.green + ", " +
                    detectedColor.blue + ") \n" +
                    "Proximity: " + getBallSensorProximity());
            return BallColor.NONE;
        }
    }
}
