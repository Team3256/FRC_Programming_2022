package frc.robot.helper.CANdle.PatternGenerators;

import frc.robot.helper.BallColor;
import frc.robot.helper.CANdle.helpers.LEDColor;
import frc.robot.helper.CANdle.helpers.HashMapFiller;
import frc.robot.helper.CANdle.helpers.LEDInstruction;

import java.util.ArrayList;
import java.util.HashMap;

import static frc.robot.Constants.PatternGeneratorConstants.*;
import static frc.robot.helper.BallColor.*;
import static java.util.Map.entry;

public class BallColorPatternGenerator implements PatternGenerator {
    private final HashMap<BallColor, LEDColor> colorToConstantColor = HashMapFiller.populateHashMap(
            entry(RED, RED_BALL_LED_COLOR),
            entry(BLUE, BLUE_BALL_LED_COLOR),
            entry(NONE, LEDColor.off)
    );

    BallColor ball1Color = BallColor.NONE;
    BallColor ball2Color = BallColor.NONE;

    public void update(BallColor ball1Color, BallColor ball2Color){
        this.ball1Color = ball1Color;
        this.ball2Color = ball2Color;
    }

    @Override
    public ArrayList<LEDInstruction> getLEDInstructions(boolean isSpoofed, int ledCount) {

        ArrayList<LEDInstruction> ledInstructions = new ArrayList<>();

        // For now, just turn off if spoofed
        if (isSpoofed) {
            ledInstructions.add(new LEDInstruction(LEDColor.off, 0, ledCount));
            return ledInstructions;
        }

        // Deal with Odd / Even LED Counts
        int ball1SectionLen = ledCount / 2;
        int ball2Sectionlen = ledCount % 2 == 0 ? ledCount / 2 : ledCount / 2 +1;

        ledInstructions.add(generateLEDInstruction(ball1Color, 0, ball1SectionLen));
        ledInstructions.add(generateLEDInstruction(ball2Color, ball1SectionLen, ball2Sectionlen));

        return ledInstructions;
    }

    private LEDInstruction generateLEDInstruction(BallColor ballColor, int startIdx, int endIdx) {
        return colorToConstantColor.get(ballColor).toLedInstruction(startIdx, endIdx);
    }
}
