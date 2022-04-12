package frc.robot.util.buttons;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * A button representation of an axis input like a trigger button that returns a numeric value
 */
public class EnhancedAxisButton extends Button {

    private boolean isRisingEdge;
    private boolean isFallingEdge;

    public EnhancedAxisButton(GenericHID hid, int rawAxis, double pressedThreshold) {
        super(() -> {
            double input = hid.getRawAxis(rawAxis);

            return Math.abs(input) >= pressedThreshold;
        });

        CommandScheduler.getInstance().addButton(
                new Runnable() {
                    private boolean previousState = get();

                    @Override
                    public void run() {
                        boolean currentState = get();

                        isRisingEdge = currentState && !previousState;
                        isFallingEdge = !currentState && previousState;

                        previousState = currentState;
                    }
                }
        );
    }

    /**
     * Returns true if the button has just been pressed.
     */
    public boolean isRisingEdge() {
        return isRisingEdge;
    }

    /**
     * Returns true if the button has just been released.
     */
    public boolean isFallingEdge() {
        return isFallingEdge;
    }

}
