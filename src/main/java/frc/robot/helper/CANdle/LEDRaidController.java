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

    boolean updateQueued = false;

    CANdle candle;

    SwerveDrive swerveDrive;

    // constructor
    public LEDRaidController(CANdle candle, SwerveDrive swerveDrive) {
        this.candle = candle;
        this.swerveDrive = swerveDrive;
    }

    /**
     * Queues an update
     */
    public void queueUpdate(){
        updateQueued = true;
    }

    /**
     * Checks if there's an update queued, if there is, run the update.
     * Should be run every cycle
     */
    public void runUpdates(){
        if (updateQueued) {
            update();
            updateQueued = false;
        }
    }

    private void update() {
        for (LEDRange range: RANGES){
            for (Section section: SECTIONS_TO_GENERATOR.keySet()){

                //Gets the instruction array from Pattern Generator
                // +0.5 needed to find closest int for a given percentage
                ArrayList<LEDInstruction> instructions =
                        SECTIONS_TO_GENERATOR.get(section)
                                .getLEDInstructions(
                                        isSpoofed(range),
                                        (int)(range.getLength() * SECTIONS_TO_PERCENTAGE.get(section) + 0.5));

                //Runs Instructions for a given Section
                for (LEDInstruction instruction: instructions){
                    candle.setLEDs(
                            instruction.color.r, instruction.color.g, instruction.color.b, instruction.color.w,
                            fromVirtualToGlobal(range, section, instruction.startIdx), instruction.count
                    );
                }
            }
        }
    }

    private boolean isSpoofed(LEDRange range){

        // 180 + Robot Heading, wrapping around if necessary
        double robotInverseHeading0to360 = swerveDrive.getPose().getRotation().getDegrees() + 180;

        if ( Math.abs(robotInverseHeading0to360 - range.degreesFromForward) <= POKERFACE_ANGLE_MARGIN_OF_ERROR)
            return false;
        else
            return true;
    }

    private int fromVirtualToGlobal(LEDRange range,Section section, int virtualAddress){

        double startingPercentage = 0;
        //Find the Starting Percentage of Range
        for (Section sectionSearch: SECTIONS_TO_GENERATOR.keySet()){
            if (section == sectionSearch){
                break;
            } else {
                startingPercentage += SECTIONS_TO_PERCENTAGE.get(sectionSearch);
            }
        }

        int sectionStartOffsetInRange = (int)(range.getLength() * startingPercentage + 0.5);

        return range.lower + sectionStartOffsetInRange + virtualAddress;
    }


}
