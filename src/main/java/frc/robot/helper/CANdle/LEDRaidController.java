package frc.robot.helper.CANdle;

// get ranges and secions from constants file

import com.ctre.phoenix.led.CANdle;
import frc.robot.helper.CANdle.helpers.LEDInstruction;
import frc.robot.helper.CANdle.helpers.LEDRange;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.logging.Logger;

import static frc.robot.Constants.POKERFACE_ANGLE_MARGIN_OF_ERROR;
import static frc.robot.Constants.CANdleConstants.*;

public class LEDRaidController {
    private static final Logger logger = Logger.getLogger(LEDRaidController.class.getCanonicalName());

    CANdle candle;

    SwerveDrive swerveDrive;

    public LEDRaidController(CANdle candle, SwerveDrive swerveDrive) {
        this.candle = candle;
        this.swerveDrive = swerveDrive;
    }

    public void update() {
        for (LEDRange range: RANGES){
            for (LEDSectionName ledSectionName : SECTIONS.keySet()){

                // Gets the instruction array from Pattern Generator
                // +0.5 needed to find closest int for a given percentage
                ArrayList<LEDInstruction> instructions =
                        SECTIONS.get(ledSectionName).patternGenerator
                                .getLEDInstructions(
                                        isSpoofed(range),
                                        (int)(range.getLength() * SECTIONS.get(ledSectionName).getPercentageRange() + 0.5));

                // Runs Instructions for a given Section
                for (LEDInstruction instruction: instructions){
                    candle.setLEDs(
                            instruction.LEDColor.r, instruction.LEDColor.g, instruction.LEDColor.b, instruction.LEDColor.w,
                            fromVirtualToGlobal(range, ledSectionName, instruction.startIdx), instruction.count
                    );
                }
            }
        }
    }

    private boolean isSpoofed(LEDRange range){

        // 180 + Robot Heading, wrapping around if necessary
        double robotInverseHeading0to360 = swerveDrive.getPose().getRotation().getDegrees() + 180;

        return !(Math.abs(robotInverseHeading0to360 - range.degreesFromForward) <= POKERFACE_ANGLE_MARGIN_OF_ERROR);
    }

    private int fromVirtualToGlobal(LEDRange range, LEDSectionName ledSectionName, int virtualAddress){
        int sectionStartOffsetInRange = (int)(range.getLength() * SECTIONS.get(ledSectionName).percentageStart + 0.5);

        return range.lower + sectionStartOffsetInRange + virtualAddress;
    }


}
