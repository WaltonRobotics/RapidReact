package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.util.EnhancedJoystickButton;
import frc.robot.util.Gamepad;

import static frc.robot.Constants.ControllerPorts.XboxConfigPorts.kDriveXboxControllerPort;
import static frc.robot.Constants.ControllerPorts.XboxConfigPorts.kManipulationXboxControllerPort;
import static frc.robot.util.Gamepad.Button.RIGHT_BUMPER;
import static frc.robot.util.Gamepad.Button.RIGHT_TRIGGER;

public class XboxConfig implements ControllerConfig{
    public static XboxController gamepad = new XboxController(kDriveXboxControllerPort);
    public static Gamepad manipulationGamepad = new Gamepad(kManipulationXboxControllerPort);

    private final ExpoScale forwardScale = new ExpoScale(0.1, 0.6);
    private final ExpoScale strafeScale = new ExpoScale(0.1, 0.6);
    private final ExpoScale yawScale = new ExpoScale(0.1, 0.75);

    private final EnhancedJoystickButton resetDrivetrainButton = new EnhancedJoystickButton(gamepad, XboxController.Button.kLeftBumper.value);
    private final EnhancedJoystickButton autoAimButton = new EnhancedJoystickButton(gamepad, XboxController.Button.kRightBumper.value);
    private final EnhancedJoystickButton intakeButton = new EnhancedJoystickButton(manipulationGamepad, RIGHT_TRIGGER.getIndex());
    private final EnhancedJoystickButton outtakeButton = new EnhancedJoystickButton(manipulationGamepad, RIGHT_BUMPER.getIndex());

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

    public EnhancedJoystickButton getAutoAimButton() {
        return autoAimButton;
    }

    @Override
    public EnhancedJoystickButton getIntakeButton() {
        return intakeButton;
    }

    @Override
    public EnhancedJoystickButton getOuttakeButton() {
        return outtakeButton;
    }

}
