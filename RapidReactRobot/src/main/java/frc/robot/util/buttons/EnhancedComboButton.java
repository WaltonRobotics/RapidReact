package frc.robot.util.buttons;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;

/**
 * Represents a combination of buttons that need to be pressed to activate
 * this button
 */
public class EnhancedComboButton extends Button {

    private boolean isRisingEdge;
    private boolean isFallingEdge;

    public EnhancedComboButton(Button... buttons) {
        super(() -> {
            for (Button button : buttons) {
                if (!button.get()) {
                    return false;
                }
            }

            return true;
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
