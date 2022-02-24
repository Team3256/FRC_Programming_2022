package frc.robot.helper;

import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.hal.simulation.I2CDataJNI;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;


import static frc.robot.Constants.HangerConstants.MAX_CONFIDENCE_DEVIATION;
import static frc.robot.Constants.HangerConstants.TAPE_COLOR;
import static frc.robot.Constants.IDConstants.*;

public class MuxedColorSensor {

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
        changeMuxPort(BALL_COLOR_SENSOR_MUX_PORT);
        leftAlignColorSensor = new ColorSensorV3(I2C.Port.kOnboard);
        changeMuxPort(BALL_COLOR_SENSOR_MUX_PORT);
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

    public boolean colorsMatch(Color color1, Color color2){
        ColorMatch colorMatcher = new ColorMatch();
        colorMatcher.addColorMatch(color2);
        ColorMatchResult result = colorMatcher.matchColor(color1);
        return (1-result.confidence)<MAX_CONFIDENCE_DEVIATION;
    }

    public boolean rightAlignSensorDetectsTape(){
        return colorsMatch(getRightAlignSensorColor(), TAPE_COLOR);
    }

    public boolean leftAlignSensorDetectsTape(){
        return colorsMatch(getLeftAlignSensorColor(), TAPE_COLOR);
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
