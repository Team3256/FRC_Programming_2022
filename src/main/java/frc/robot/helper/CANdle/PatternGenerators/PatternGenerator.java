package frc.robot.helper.CANdle.PatternGenerators;

import frc.robot.helper.CANdle.helpers.LEDInstruction;

import java.util.ArrayList;

public interface PatternGenerator {
    ArrayList<LEDInstruction> getLEDInstructions(boolean isSpoofed, int ledCount);
}
