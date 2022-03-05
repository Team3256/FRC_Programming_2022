package frc.robot.helper.LED.helpers;
import frc.robot.helper.logging.RobotLogger;

import java.util.logging.Logger;

public class LEDRange {
    private static final RobotLogger logger = new RobotLogger(LEDRange.class.getCanonicalName());
    /**
     * Inclusive Index of LED
     */
    public int lower;

    /**
     * Exclusive Index of LED
     */
    public int upper;

    /**
     * Degrees of range from the forward direction (+ is clockwise)
     */
    public double degreesFromForward;

    /**
     * @param lower Inclusive Index of LED
     * @param upper Exclusive Index of LED
     * @param degreesFromForward Degrees of range from the forward direction (+ is clockwise)
     */
    public LEDRange(int lower, int upper, double degreesFromForward) {
        if (lower > upper) {
            logger.warning(String.format("Lower bound %d is greater than upper bound %d, assigning both to upper", lower, upper));
            this.lower = upper;
            this.upper = upper;
        } else {
            this.lower = lower;
            this.upper = upper;
        }
        this.degreesFromForward = degreesFromForward;
    }
    public int getLength(){
        return upper-lower;
    }
}
