package frc.robot.hardware;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.helper.BallColor;
import frc.robot.subsystems.TransferSubsystem;


import java.util.logging.Logger;

import static frc.robot.Constants.HangerConstants.*;
import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.TransferConstants.*;

public class MuxedColorSensor {
    private static final Logger logger = Logger.getLogger(MuxedColorSensor.class.getCanonicalName());

    //Singleton Design Pattern
    private static MuxedColorSensor instance;
    public static MuxedColorSensor getInstance() {
        if (instance == null)
            instance = new MuxedColorSensor();
        return instance;
    }

    private final ColorSensorV3 ballColorSensor;
    private final ColorSensorV3 leftAlignColorSensor;
    private final ColorSensorV3 rightAlignColorSensor;

    private MuxedColorSensor(){
        changeMuxPort(BALL_COLOR_SENSOR_MUX_PORT);
        ballColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        changeMuxPort(LEFT_ALIGN_COLOR_SENSOR_MUX_PORT);
        leftAlignColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        changeMuxPort(RIGHT_ALIGN_COLOR_SENSOR_MUX_PORT);
        rightAlignColorSensor = new ColorSensorV3(I2C.Port.kOnboard);


    }

    public Color getBallSensorColor() {
        changeMuxPort(BALL_COLOR_SENSOR_MUX_PORT);
        return ballColorSensor.getColor();
    }

    public int getBallSensorProximity() {
        changeMuxPort(BALL_COLOR_SENSOR_MUX_PORT);
        return ballColorSensor.getProximity();
    }

    public Color getLeftAlignSensorColor() {
        changeMuxPort(LEFT_ALIGN_COLOR_SENSOR_MUX_PORT);
        return leftAlignColorSensor.getColor();
    }

    public Color getRightAlignSensorColor(){
        changeMuxPort(RIGHT_ALIGN_COLOR_SENSOR_MUX_PORT);
        return rightAlignColorSensor.getColor();
    }

    private boolean colorsMatch(Color color1, Color color2, double maxConfidenceDeviation){
        ColorMatch colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(color2);
        ColorMatchResult result = colorMatcher.matchColor(color1);
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

        if (isRedBallDetected)
            return BallColor.RED;
        else if (isBlueBallDetected)
            return BallColor.BLUE;
        else
            return BallColor.NONE;
    }

    public boolean leftAlignSensorDetectsTape(){
        return colorsMatch(getLeftAlignSensorColor(), TAPE_COLOR, MAX_TAPE_COLOR_CONFIDENCE_DEVIATION);
    }

    public boolean rightAlignSensorDetectsTape(){
        return colorsMatch(getRightAlignSensorColor(), TAPE_COLOR, MAX_TAPE_COLOR_CONFIDENCE_DEVIATION);
    }

    /**
     * Changes the Mux to Select certain I2C port
     * @param port Port from 0..9
     */
    private void changeMuxPort(int port){
        //Writes Bit mask corresponding with the port selected to the MUX address
        I2CJNI.i2CWriteB(I2C.Port.kOnboard.value, I2C_MUX_ADDRESS, new byte[]{(byte) (0x1 << port)}, (byte) 0x1);
    }




}
