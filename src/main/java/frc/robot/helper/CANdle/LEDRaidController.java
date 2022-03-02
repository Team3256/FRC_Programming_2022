package frc.robot.helper.CANdle;

// get ranges and secions from constants file

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.helper.CANdle.helpers.LEDInstruction;
import frc.robot.helper.CANdle.helpers.LEDRange;
import frc.robot.helper.CANdle.helpers.LEDSectionAttributes;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.logging.Logger;

import static frc.robot.Constants.POKERFACE_ANGLE_MARGIN_OF_ERROR;
import static frc.robot.Constants.CANdleConstants.*;

public class LEDRaidController {

    private static final Logger logger = Logger.getLogger(LEDRaidController.class.getCanonicalName());
    private LinkedList<LEDInstruction> ledInstructionLinkedList = new LinkedList<>();

    CANdle candle;
    Timer timer = new Timer();

    private int maxLeds = 0;

    private boolean isEnabled = true;

    SwerveDrive swerveDrive;

    public LEDRaidController(CANdle candle, SwerveDrive swerveDrive) {
        this.candle = candle;
        this.swerveDrive = swerveDrive;

        CANdleConfiguration candleConfig = new CANdleConfiguration();
        candleConfig.stripType = LED_STRIP_TYPE;
        candleConfig.brightnessScalar = LED_BRIGHTNESS_SCALAR;

        candle.configAllSettings(candleConfig);

        // Zero Out all LEDs on Startup
        for (LEDRange range: RANGES){
            if (maxLeds < range.upper)
                maxLeds = range.upper;
        }
        ledInstructionLinkedList.add(new LEDInstruction(0,0,0,0,0, maxLeds));

        timer.start();
    }

    public void update() {

        if(DriverStation.isDisabled() && isEnabled) {  // Just got Disabled
            isEnabled = false;
            disabledLights();
            return;

        } else if (DriverStation.isDisabled() && !isEnabled){  // Robot Is Disabled
            return;

        } else if (!isEnabled) {  // Just Enabled
            isEnabled = true;
            for (LEDSectionName ledSectionName : SECTIONS.keySet())
                SECTIONS.get(ledSectionName).patternGenerator.reset();

            candle.setLEDs(0,0,0,0, 0, maxLeds);  // Turn off LEDs
            timer.reset();  // Wait for Cooldown
        }

        // Wait if running too quickly
        if(!timer.hasElapsed(MIN_WAIT_TIME_BETWEEN_INSTRUCTIONS)) {
            return;
        }


        // Check Pattern Generators for Updates ------

        // Check what Sections need Updating
        LinkedHashMap<LEDSectionName, LEDSectionAttributes> sectionsNeedUpdate = new LinkedHashMap<>();
        for (LEDSectionName ledSectionName : SECTIONS.keySet()){
            if (SECTIONS.get(ledSectionName).patternGenerator.shouldUpdate())
                sectionsNeedUpdate.put(ledSectionName, SECTIONS.get(ledSectionName));
        }

        // Update Sections + Get Instructions, Put into FIFO Queue
        for (LEDRange range: RANGES){
            for (LEDSectionName ledSectionName : sectionsNeedUpdate.keySet()){

                // Get Instructions from Pattern
                // +0.5 needed to find the closest int for a given percentage
                ArrayList<LEDInstruction> instructions =
                        SECTIONS.get(ledSectionName).patternGenerator
                                .getLEDInstructions(
                                        isSpoofed(range),
                                        (int)(range.getLength() * SECTIONS.get(ledSectionName).getPercentageRange() + 0.5));

                // Adds Instructions to FIFO Queue
                for (LEDInstruction instruction: instructions) {

                    // If big, Splits up Instructions greater than 120 leds, since CANdle can't handle it
                    if (instruction.count > MAX_LED_INSTRUCTION_BLOCK_SIZE){
                        addSplitInstructions(instruction, range, ledSectionName);
                    } else {
                        ledInstructionLinkedList.add(new LEDInstruction(
                                instruction.ledColor,
                                fromVirtualToGlobal(range, ledSectionName, instruction.startIdx),
                                instruction.count));
                    }
                }
            }
        }

        // Actually Run CANdle Commands, 1 at a time
        LEDInstruction caNdleInstruction = ledInstructionLinkedList.pollFirst();

        if (caNdleInstruction != null) {
            ErrorCode error = candle.setLEDs(
                    caNdleInstruction.ledColor.r, caNdleInstruction.ledColor.g,
                    caNdleInstruction.ledColor.b, caNdleInstruction.ledColor.w,
                    caNdleInstruction.startIdx, caNdleInstruction.count

                    );

            if (!error.equals(ErrorCode.OK))
                logger.warning(error.toString());
        }

        timer.reset();
    }

    private void disabledLights(){
        // Just turn off Lights when Disabled
        candle.setLEDs(0,0,0,0, 0, maxLeds);
    }

    private boolean isSpoofed(LEDRange range){

        // 180 + Robot Heading, wrapping around if necessary
        double robotInverseHeading0to360 = swerveDrive.getPose().getRotation().getDegrees() + 180;

        return !(Math.abs(robotInverseHeading0to360 - range.degreesFromForward) <= POKERFACE_ANGLE_MARGIN_OF_ERROR);
    }

    private void addSplitInstructions (LEDInstruction instruction, LEDRange range, LEDSectionName ledSectionName){
        int startIdx = instruction.startIdx;
        int count = MAX_LED_INSTRUCTION_BLOCK_SIZE;

        while (true){

            ledInstructionLinkedList.add(new LEDInstruction(
                    instruction.ledColor,
                    fromVirtualToGlobal(range, ledSectionName, startIdx),
                    count));


            startIdx += MAX_LED_INSTRUCTION_BLOCK_SIZE;

            // If at end of Instruction, Make last one shorter
            if (instruction.count - startIdx < MAX_LED_INSTRUCTION_BLOCK_SIZE ) {
                count -= instruction.count - startIdx;
            }

            // If out of range, we're done
            if (startIdx > instruction.count){
                return;
            }

        }
    }


    private int fromVirtualToGlobal(LEDRange range, LEDSectionName ledSectionName, int virtualAddress){
        int sectionStartOffsetInRange = (int)(range.getLength() * SECTIONS.get(ledSectionName).percentageStart + 0.5);

        return range.lower + sectionStartOffsetInRange + virtualAddress;
    }


}
