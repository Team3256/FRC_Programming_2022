package frc.robot.helper;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.hal.I2CJNI;
import edu.wpi.first.hal.simulation.I2CDataJNI;
import edu.wpi.first.wpilibj.I2C;

import java.awt.*;

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

    /**
     * Use ColorSensor Immediately after getting, may not be valid for long
     */
    public ColorSensorV3 getBallColorSensor() {
        changeMuxPort(BALL_COLOR_SENSOR_MUX_PORT);
        return ballColorSensor;
    }

    /**
     * Use ColorSensor Immediately after getting, may not be valid for long
     */
    public ColorSensorV3 getLeftAlignColorSensor() {
        changeMuxPort(LEFT_ALIGN_COLOR_SENSOR_MUX_PORT);
        return leftAlignColorSensor;
    }

    /**
     * Use ColorSensor Immediately after getting, may not be valid for long
     */
    public ColorSensorV3 getRightAlignColorSensor(){
        changeMuxPort(RIGHT_ALIGN_COLOR_SENSOR_MUX_PORT);
        return rightAlignColorSensor;
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
