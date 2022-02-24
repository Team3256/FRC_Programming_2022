package frc.robot.helper.CANdle.PatternGenerators;

import frc.robot.helper.CANdle.helpers.LEDColor;
import frc.robot.helper.CANdle.helpers.LEDInstruction;

import java.util.ArrayList;

import static frc.robot.Constants.PatternGeneratorConstants.AUTO_AIM_LED_COLOR;

public class AutoAimPatternGenerator implements PatternGenerator {

    private boolean isAutoAiming = false;
    private int counter;

    private boolean ledsOn;

    @Override
    public ArrayList<LEDInstruction> getLEDInstructions(boolean isSpoofed, int ledCount) {
        counter = (counter + 1 ) % 8;
        if(counter != 0)
            return new ArrayList<>();

        ledsOn = !ledsOn;

        ArrayList<LEDInstruction> ledInstructions = new ArrayList<>();

        ledInstructions.add((ledsOn ? AUTO_AIM_LED_COLOR : LEDColor.off).toLedInstruction(0, ledCount));

        return ledInstructions;

    }

    public void update(boolean isAutoAiming){
        this.isAutoAiming = isAutoAiming;
    }
}
