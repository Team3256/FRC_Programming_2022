package frc.robot.helper.LED.PatternGenerators;

import edu.wpi.first.wpilibj.Timer;
import frc.robot.helper.LED.helpers.LEDColor;
import frc.robot.helper.LED.helpers.LEDInstruction;

import java.util.ArrayList;

import static frc.robot.Constants.PatternGeneratorConstants.AUTO_AIM_LED_COLOR;

public class AutoAimPatternGenerator implements PatternGenerator {

    private boolean isAutoAiming;
    private Timer timer = new Timer();

    private boolean ledsOn;

    public AutoAimPatternGenerator(){
        reset();
    }

    @Override
    public boolean shouldUpdate() {
        if (timer.hasElapsed(1))
            ledsOn = !ledsOn;
        return timer.hasElapsed(1);
    }

    @Override
    public void reset() {
        timer.reset();
        timer.start();
        isAutoAiming = true;
    }

    @Override
    public ArrayList<LEDInstruction> getLEDInstructions(boolean isSpoofed, int ledCount) {


        System.out.println(timer.get());
        timer.reset();
        timer.start();


        ArrayList<LEDInstruction> ledInstructions = new ArrayList<>();

        if (!isAutoAiming) {
            ledInstructions.add((AUTO_AIM_LED_COLOR).toLedInstruction(0, ledCount));
            return ledInstructions;
        }



        ledInstructions.add((ledsOn ? AUTO_AIM_LED_COLOR : LEDColor.off).toLedInstruction(0, ledCount));

        return ledInstructions;

    }

    public void update(boolean isAutoAiming){
        this.isAutoAiming = isAutoAiming;
    }
}
