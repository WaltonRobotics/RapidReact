package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.util.WPIUtilJNI;

public class DecelLimiter {

    private double m_prevVal;
    private double m_prevTime;

    /**
     * Creates a new DecelLimiter with the given rate limit and initial value.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     * @param initialValue The initial value of the input.
     */
    public DecelLimiter(double rateLimit, double initialValue) {
        m_rateLimit = rateLimit;
        m_prevVal = initialValue;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
     * Creates a new DecelLimiter with the given rate limit and an initial value of zero.
     *
     * @param rateLimit The rate-of-change limit, in units per second.
     */
    public DecelLimiter(double rateLimit) {
        this(rateLimit, 0);
    }

    /**
     * Filters the input to limit its slew rate.
     *
     * @param input The input value whose slew rate is to be limited.
     * @return The filtered value, which will not change faster than the slew rate.
     */
    public double calculate(double input) {
        double currentTime = WPIUtilJNI.now() * 1e-6;
        double elapsedTime = currentTime - m_prevTime;

        m_prevVal += Math.max(input - m_prevVal, -m_rateLimit * elapsedTime);

        m_prevTime = currentTime;
        return m_prevVal;
    }

    /**
     * Resets the slew rate limiter to the specified value; ignores the rate limit when doing so.
     *
     * @param value The value to reset to.
     */
    public void reset(double value) {
        m_prevVal = value;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

}
