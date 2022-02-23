package frc.robot.helper.CANdle.PatternGenerators;

import frc.robot.helper.CANdle.LEDInstruction;

import java.util.ArrayList;
import java.util.HashMap;

public interface PatternGenerator {
    ArrayList<LEDInstruction> getLEDInstructions(boolean isSpoofed, int ledCount);
}
