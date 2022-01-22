package frc.robot.util;

public class WaltTimesliceRobot extends WaltTimedRobot {

    /**
     * Constructor for WaltTimesliceRobot.
     *
     * @param robotPeriodicAllocation The allocation in seconds to give the TimesliceRobot periodic
     *     functions.
     * @param controllerPeriod The controller period in seconds. The sum of all scheduler allocations
     *     should be less than or equal to this value.
     */
    public WaltTimesliceRobot(double robotPeriodicAllocation, double controllerPeriod) {
        m_nextOffset = robotPeriodicAllocation;
        m_controllerPeriod = controllerPeriod;
    }

    /**
     * Schedule a periodic function with the constructor's controller period and the given allocation.
     * The function's runtime allocation will be placed after the end of the previous one's.
     *
     * <p>If a call to this function makes the allocations exceed the controller period, an exception
     * will be thrown since that means the TimesliceRobot periodic functions and the given function
     * will have conflicting timeslices.
     *
     * @param func Function to schedule.
     * @param allocation The function's runtime allocation in seconds out of the controller period.
     */
    public void schedule(Runnable func, double allocation) {
        if (m_nextOffset + allocation > m_controllerPeriod) {
            throw new IllegalStateException(
                    "Function scheduled at offset "
                            + m_nextOffset
                            + " with allocation "
                            + allocation
                            + " exceeded controller period of "
                            + m_controllerPeriod
                            + "\n");
        }

        addPeriodic(func, m_controllerPeriod, m_nextOffset);
        m_nextOffset += allocation;
    }

    private double m_nextOffset;
    private final double m_controllerPeriod;

}
