package frc.robot.util.buttons;

import java.util.Arrays;
import java.util.List;

public class MultiButtonCombo {

    private final List<EnhancedJoystickButton> buttons;

    public MultiButtonCombo(EnhancedJoystickButton... comboButtons) {
        buttons = Arrays.asList(comboButtons);
    }

    public boolean get() {
        for (EnhancedJoystickButton button : buttons) {
            if (!button.get()) {
                return false;
            }
        }

        return true;
    }

}
