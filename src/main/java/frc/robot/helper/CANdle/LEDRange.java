package frc.robot.helper.CANdle;
import java.util.logging.Logger;

public class LEDRange {
    private static final Logger logger = Logger.getLogger(LEDRange.class.getCanonicalName());
    /**
     * Inclusive Index of LED
     */
    public int lower;

    /**
     * Exclusive Index of LED
     */
    public int upper;

    /**
     * @param lower Inclusive Index of LED
     * @param upper Exclusive Index of LED
     */
    public LEDRange(int lower, int upper) {
        if (lower > upper) {
            logger.warning(String.format("Lower bound %d is greater than upper bound %d, assigning both to upper", lower, upper));
            this.lower = upper;
            this.upper = upper;
        } else {
            this.lower = lower;
            this.upper = upper;
        }
    }
}
