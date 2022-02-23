package frc.robot.helper.CANdle;

// get ranges and secions from constants file

import com.ctre.phoenix.led.CANdle;
import frc.robot.helper.CANdle.PatternGenerators.PatternGenerator;

import java.util.Map;

import static frc.robot.Constants.CANdleConstants.*;

import static frc.robot.Constants.CANdleConstants.SECTIONS_TO_GENERATOR;

public class LEDRaidController {

    CANdle candle;

    // constructor
    public LEDRaidController(CANdle candle) {
        this.candle = candle;
    }

    public void update() {

    }

    public boolean isSpoofed(LEDRange range){
        //Does Math to calc if spoofed based on rotation
        return false;
    }


}
