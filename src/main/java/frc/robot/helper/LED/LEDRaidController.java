package frc.robot.helper.LED;

// get ranges and secions from constants file

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.helper.LED.helpers.LEDInstruction;
import frc.robot.helper.LED.helpers.LEDRange;
import frc.robot.helper.LED.helpers.LEDSectionAttributes;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.logging.Logger;

import static frc.robot.Constants.IDConstants.LED_STRIP_PWM_PORT;
import static frc.robot.Constants.LEDConstants.*;

public class LEDRaidController {
    private static final Logger logger = Logger.getLogger(LEDRaidController.class.getCanonicalName());

    private AddressableLED addressableLED;
    // This is the global LED buffer, whatever is in here will be pushed out to LEDs
    private AddressableLEDBuffer buffer;

    // Enforces the wait time between commands
    Timer waitTimer = new Timer();

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

        waitTimer.start();

        // Start Sending Values to LEDs
        addressableLED.start();

        // Stuff when Disabling vs Enabling
        new Trigger(DriverStation::isDisabled).whenActive(()->{
            disabledLights();
            waitTimer.reset();
            waitTimer.stop();
        });
        new Trigger(DriverStation::isEnabled).whenActive(()->{
            // Figure out which Patterns need updating
            for (LEDSectionName ledSectionName : SECTIONS.keySet())
                SECTIONS.get(ledSectionName).patternGenerator.reset();

            waitTimer.start();
        });
    }

    public void update() {
        // Wait if running too quickly / Disabled (Since we reset timer)
        if(!waitTimer.hasElapsed(MIN_WAIT_TIME_BETWEEN_INSTRUCTIONS)) {
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


                // TODO: This entire section needs reworking


                // First we need to Create a Buffer (Array of LedColor) that represents the LED Section we want to write too
                // Size: (int)(range.getLength() * SECTIONS.get(ledSectionName).getPercentageRange() + 0.5)

                // Then We need to pass that buffer to each pattern generator
                // (Which we need to modify so that works)
                // to modify that buffer instead of creating a new LEDInstruction Array
                // TODO: Modify each Pattern Generator so it modifies the current Array it was given, not creating a new array


                // Lastly we need to transform the "Local Space" (Array 0-size of whatever that section is) into
                // "Global Space" which is a section of the entire LED strip

                // But here is the mapping
                // let GlobalStartIdx = fromVirtualIndexToGlobalIndex(range, ledSectionName, instruction.startIdx)

                // 0  to  size -> Local Space
                // GlobalStartIdx   to   size + GlobalStartIdx -> Global Space


                // Code down here is all obsolete but is here for reference ------------------------------------

                // Get Instructions from Pattern
                // +0.5 needed to find the closest int for a given percentage
                ArrayList<LEDInstruction> instructions =
                        SECTIONS.get(ledSectionName).patternGenerator
                                .getLEDInstructions(
                                        isSpoofed(range),
                                        (int)(range.getLength() * SECTIONS.get(ledSectionName).getPercentageRange() + 0.5));

                // Adds Instructions to FIFO Queue
                for (LEDInstruction instruction: instructions) {

                    int startIdx = fromVirtualIndexToGlobalIndex(range, ledSectionName, instruction.startIdx);

                    for (int i = startIdx; i < startIdx + instruction.count; i++) {
                        buffer.setRGB(
                                i, instruction.ledColor.r, instruction.ledColor.g,
                                instruction.ledColor.b);
                    }
                }
            }
        }

        addressableLED.setData(buffer);
        waitTimer.reset();
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

    /**
     * Transforms the Virtual Address Space (0-led section count) to global (0 - max led count).
     * This is based on the sections and ranges defined in Constants.java
     *
     * @param range Range of virtual address to transform
     * @param ledSectionName Section of virtual address to transform
     * @param virtualAddress Start Index of address to transform
     * @return Global Start Index to transform
     */
    private int fromVirtualIndexToGlobalIndex(LEDRange range, LEDSectionName ledSectionName, int virtualAddress){
        int sectionStartOffsetInRange = (int)(range.getLength() * SECTIONS.get(ledSectionName).percentageStart + 0.5);

        return range.lower + sectionStartOffsetInRange + virtualAddress;
    }


}
