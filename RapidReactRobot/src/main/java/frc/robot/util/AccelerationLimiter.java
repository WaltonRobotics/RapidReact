package frc.robot.util;

import edu.wpi.first.util.WPIUtilJNI;

public class AccelerationLimiter {

    private final double m_accelerationRateLimit;
    private final double m_decelerationRateLimit;
    private double m_prevVal;
    private double m_prevTime;

    /**
     * Creates a new AccelerationLimiter with the given rate limit and initial value.
     */
    public AccelerationLimiter(double accelerationRateLimit, double decelerationRateLimit, double initialValue) {
        m_accelerationRateLimit = accelerationRateLimit;
        m_decelerationRateLimit = decelerationRateLimit;
        m_prevVal = initialValue;
        m_prevTime = WPIUtilJNI.now() * 1e-6;
    }

    /**
     * Creates a new DecelLimiter with the given rate limit and an initial value of zero.
     */
    public AccelerationLimiter(double accelerationRateLimit, double decelerationRateLimit) {
        this(accelerationRateLimit, decelerationRateLimit, 0);
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

        if (m_prevVal < 0) {
            m_prevVal += UtilMethods.limitRange(input - m_prevVal, -m_accelerationRateLimit * elapsedTime,
                    m_decelerationRateLimit * elapsedTime);
        } else {
            m_prevVal += UtilMethods.limitRange(input - m_prevVal, -m_decelerationRateLimit * elapsedTime,
                    m_accelerationRateLimit * elapsedTime);
        }

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
