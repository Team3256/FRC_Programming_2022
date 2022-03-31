package frc.robot.helper;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

public class AdaptiveSlewRateLimiter {
    private final double accelRateLimit;
    private final double deccelRateLimit;
    private double prevVal;
    private double prevTime;

    /**
     * Creates a new SlewRateLimiter with the given rate limit and initial value.
     *
     * @param accelRateLimit The rate-of-change limit, in units per second.
     * @param deccelRateLimit The rate-of-change limit, in units per second.
     */
    public AdaptiveSlewRateLimiter(double accelRateLimit, double deccelRateLimit) {
        this.accelRateLimit = Math.abs(accelRateLimit);
        this.deccelRateLimit = Math.abs(deccelRateLimit);
        prevVal = 0;
        prevTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - prevTime;
        double currRateLimit = (Math.abs(input) > prevVal ? accelRateLimit : deccelRateLimit);
        prevVal +=
                MathUtil.clamp(input - prevVal, -currRateLimit * elapsedTime, currRateLimit * elapsedTime);
        prevTime = currentTime;
        return prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        prevVal = value;
        prevTime = WPIUtilJNI.now() * 1e-6;
    }
}
