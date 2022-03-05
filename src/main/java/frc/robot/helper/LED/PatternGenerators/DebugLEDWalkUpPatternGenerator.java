package frc.robot.helper.LED.PatternGenerators;

import frc.robot.helper.LED.helpers.LEDColor;
import frc.robot.helper.LED.helpers.LEDInstruction;

import java.util.ArrayList;

public class DebugLEDWalkUpPatternGenerator implements PatternGenerator{

    private boolean didRun = false;

    @Override
    public ArrayList<LEDInstruction> getLEDInstructions(boolean isSpoofed, int ledCount) {

        ArrayList<LEDInstruction> ledInstructions = new ArrayList<>();

        for(int i = 0; i < ledCount; i++){
            ledInstructions.add(new LEDInstruction(LEDColor.fromRGB( (int) (i / 500.0 * 255.0),(int) ( 500.0 / (i + 1) * 255.0),(int) (Math.sin((double)i * 500.0) * 255)), i, 1));
        }

        return ledInstructions;
    }

    @Override
    public boolean shouldUpdate() {
        if (!didRun) {
            didRun = true;
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void reset() {
        didRun = false;
    }
}
