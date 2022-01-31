package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.util.EnhancedJoystickButton;

public class XboxConfig implements ControllerConfig{
    public static XboxController gamepad = new XboxController(0);
    public static XboxController manipulationGamepad = new XboxController(1);

    private final ExpoScale forwardScale = new ExpoScale(0.1, 0.6);
    private final ExpoScale strafeScale = new ExpoScale(0.1, 0.6);
    private final ExpoScale yawScale = new ExpoScale(0.1, 0.75);

    private final EnhancedJoystickButton resetDrivetrainButton = new EnhancedJoystickButton(gamepad, XboxController.Button.kLeftBumper.value);

    @Override
    public double getForward() {
        return -gamepad.getLeftY();
    }

    @Override
    public double getStrafe() {
        return -gamepad.getLeftX();
    }

    @Override
    public double getYaw() {
        return -gamepad.getRightX();
    }

    @Override
    public ExpoScale getForwardScale() {
        return forwardScale;
    }

    @Override
    public ExpoScale getStrafeScale() {
        return strafeScale;
    }

    @Override
    public ExpoScale getYawScale() {
        return yawScale;
    }

    @Override
    public EnhancedJoystickButton getResetDrivetrainButton() {
        return resetDrivetrainButton;
    }

}
