package frc.robot.helper.LED;

// get ranges and secions from constants file

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.helper.LED.helpers.LEDInstruction;
import frc.robot.helper.LED.helpers.LEDRange;
import frc.robot.helper.LED.helpers.LEDSectionAttributes;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.LinkedList;
import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants.LED_STRIP_PWM_PORT;
import static frc.robot.Constants.LEDConstants.*;

public class LEDRaidController {
    private static final Logger logger = Logger.getLogger(LEDRaidController.class.getCanonicalName());

    private AddressableLED addressableLED;
    private AddressableLEDBuffer buffer;

    Timer timer = new Timer();

    private int maxLeds = 0;

    private boolean isEnabled = true;

    SwerveDrive swerveDrive;

    public LEDRaidController(SwerveDrive swerveDrive) {


        // Get Max Num of LEDs
        for (LEDRange range: RANGES){
            if (maxLeds < range.upper)
                maxLeds = range.upper;
        }

        addressableLED = new AddressableLED(LED_STRIP_PWM_PORT);
        addressableLED.setLength(maxLeds);
        buffer = new AddressableLEDBuffer(maxLeds);
        addressableLED.setData(buffer);

        this.swerveDrive = swerveDrive;

        timer.start();

        // Start Sending Values to LEDs
        addressableLED.start();
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

            disabledLights(); // Turn off LEDs
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

                    int startIdx = fromVirtualToGlobal(range, ledSectionName, instruction.startIdx);

                    for (int i = startIdx; i < startIdx + instruction.count; i++) {
                        buffer.setRGB(
                                i, instruction.ledColor.r, instruction.ledColor.g,
                                instruction.ledColor.b);
                    }
                }
            }
        }

        addressableLED.setData(buffer);
        timer.reset();
    }

    private void disabledLights(){
        // Just turn off Lights when Disabled
        for (int i = 0; i < maxLeds; i++)
            buffer.setRGB(i, 0,0,0);  // Turn off LEDs
    }

    private boolean isSpoofed(LEDRange range){
        return false;
        // 180 + Robot Heading, wrapping around if necessary
    }

    private int fromVirtualToGlobal(LEDRange range, LEDSectionName ledSectionName, int virtualAddress){
        int sectionStartOffsetInRange = (int)(range.getLength() * SECTIONS.get(ledSectionName).percentageStart + 0.5);

        return range.lower + sectionStartOffsetInRange + virtualAddress;
    }


}
