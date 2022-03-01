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
    public static Timer testTimer = new Timer();

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

        if(DriverStation.isDisabled() && isEnabled) {
            isEnabled = false;
            // TODO: Disabled Lights go here
            candle.setLEDs(0,0,0,0, 0, maxLeds);
            return;
        } else if (DriverStation.isDisabled() && !isEnabled){
            return;
        } else if (!isEnabled) {
            // Just Enabled
            isEnabled = true;
            for (LEDSectionName ledSectionName : SECTIONS.keySet()){
                SECTIONS.get(ledSectionName).patternGenerator.reset();
            }
        }

        if(!timer.hasElapsed(0.03)) {
            return;
        }
        testTimer.reset();
        testTimer.start();
        LinkedHashMap<LEDSectionName, LEDSectionAttributes> sectionsNeedUpdate = new LinkedHashMap<>();
        for (LEDSectionName ledSectionName : SECTIONS.keySet()){
            if (SECTIONS.get(ledSectionName).patternGenerator.shouldUpdate())
                sectionsNeedUpdate.put(ledSectionName, SECTIONS.get(ledSectionName));
        }

        for (LEDRange range: RANGES){
            for (LEDSectionName ledSectionName : sectionsNeedUpdate.keySet()){

                // Gets the instruction array from Pattern Generator
                // +0.5 needed to find closest int for a given percentage
                ArrayList<LEDInstruction> instructions =
                        SECTIONS.get(ledSectionName).patternGenerator
                                .getLEDInstructions(
                                        isSpoofed(range),
                                        (int)(range.getLength() * SECTIONS.get(ledSectionName).getPercentageRange() + 0.5));

                // Runs Instructions for a given Section
                for (LEDInstruction instruction: instructions) {
                    ledInstructionLinkedList.add(new LEDInstruction(instruction.ledColor, fromVirtualToGlobal(range, ledSectionName, instruction.startIdx), instruction.count));
                }
            }
        }

        LEDInstruction caNdleInstruction = ledInstructionLinkedList.pollFirst();

        if (caNdleInstruction != null) {

            System.out.println(String.format("Setting CAN to: R:%d, G: %d, B:%d| StartIdx: %d, Count: %d",caNdleInstruction.ledColor.r, caNdleInstruction.ledColor.g,
                    caNdleInstruction.ledColor.b, caNdleInstruction.ledColor.w,
                    caNdleInstruction.startIdx, caNdleInstruction.count));
            ErrorCode error = candle.setLEDs(
                    caNdleInstruction.ledColor.r, caNdleInstruction.ledColor.g,
                    caNdleInstruction.ledColor.b, caNdleInstruction.ledColor.w,
                    caNdleInstruction.startIdx, caNdleInstruction.count

                    );



            if (!error.equals(ErrorCode.OK))
                System.out.println(error);
        }

        timer.reset();
        timer.start();
        if (testTimer.get() > 0.01)
            System.out.println("TestTimer!" + testTimer.get());
    }

    private boolean isSpoofed(LEDRange range){

        // 180 + Robot Heading, wrapping around if necessary
        double robotInverseHeading0to360 = 0 + 180;

        return !(Math.abs(robotInverseHeading0to360 - range.degreesFromForward) <= POKERFACE_ANGLE_MARGIN_OF_ERROR);
    }

    private int fromVirtualToGlobal(LEDRange range, LEDSectionName ledSectionName, int virtualAddress){
        int sectionStartOffsetInRange = (int)(range.getLength() * SECTIONS.get(ledSectionName).percentageStart + 0.5);

        return range.lower + sectionStartOffsetInRange + virtualAddress;
    }


}
