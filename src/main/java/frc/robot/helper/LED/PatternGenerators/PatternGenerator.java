package frc.robot.helper.LED.PatternGenerators;

import frc.robot.helper.LED.helpers.LEDInstruction;

import java.util.ArrayList;

public interface PatternGenerator {
    ArrayList<LEDInstruction> getLEDInstructions(boolean isSpoofed, int ledCount);
    boolean shouldUpdate();
    void reset();
}
