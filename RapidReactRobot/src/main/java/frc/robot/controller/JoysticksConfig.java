package frc.robot.controller;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.Constants;
import frc.robot.util.EnhancedJoystickButton;
import frc.robot.util.Gamepad;

public class JoysticksConfig implements ControllerConfig {

    private Joystick leftJoystick = new Joystick(Constants.ControllerPorts.JoysticksConfigPorts.kLeftJoystickPort);
    private Joystick rightJoystick = new Joystick(Constants.ControllerPorts.JoysticksConfigPorts.kRightJoystickPort);
    private Gamepad manipulationGamepad = new Gamepad(Constants.ControllerPorts.JoysticksConfigPorts.kManipulationGamepadPort);

    private final ExpoScale forwardScale = new ExpoScale(0.05, 0.6);
    private final ExpoScale strafeScale = new ExpoScale(0.05, 0.6);
    private final ExpoScale yawScale = new ExpoScale(0.01, 0.75);

    @Override
    public double getForward() {
        return 0;
    }

    @Override
    public double getStrafe() {
        return 0;
    }

    @Override
    public double getYaw() {
        return 0;
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
        return null;
    }

}
