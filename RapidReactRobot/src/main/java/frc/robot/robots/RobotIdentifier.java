package frc.robot.robots;

/**
 * Uses the DIO ports on the rio to identify the current robot.
 */

public enum RobotIdentifier {

    PRACTICE_RAPID_REACT(true, true, new PracticeRapidReact()),
    SWERVE_TESTBED(false, true, new SwerveTestbed()),
    COMP_RAPID_REACT(false, false, new CompRapidReact());

    private final boolean input1;
    private final boolean input2;

    WaltRobot selectedRobot;

    RobotIdentifier(boolean input1, boolean input2, WaltRobot robot) {
        this.input1 = input1;
        this.input2 = input2;
        this.selectedRobot = robot;
    }

    public static RobotIdentifier findByInputs(boolean input1, boolean input2) {
        for (RobotIdentifier i : values()) {
            if (i.input1 == input1 && i.input2 == input2) {
                return i;
            }
        }

        return COMP_RAPID_REACT;
    }

    public WaltRobot getSelectedRobot() {
        return selectedRobot;
    }

}