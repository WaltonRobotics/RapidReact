package frc.robot.controller;

import edu.wpi.first.wpilibj.Joystick;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.Constants;
import frc.robot.util.EnhancedJoystickButton;
import frc.robot.util.Gamepad;

import static frc.robot.util.Gamepad.Button.RIGHT_BUMPER;
import static frc.robot.util.Gamepad.Button.RIGHT_TRIGGER;

public class JoysticksConfig implements ControllerConfig {

    private final Joystick leftJoystick = new Joystick(Constants.ControllerPorts.JoysticksConfigPorts.kLeftJoystickPort);
    private final Joystick rightJoystick = new Joystick(Constants.ControllerPorts.JoysticksConfigPorts.kRightJoystickPort);
    private final Gamepad manipulationGamepad = new Gamepad(Constants.ControllerPorts.JoysticksConfigPorts.kManipulationGamepadPort);

    private final EnhancedJoystickButton intakeButton = new EnhancedJoystickButton(manipulationGamepad,RIGHT_TRIGGER.getIndex());
    private final EnhancedJoystickButton outtakeButton = new EnhancedJoystickButton(manipulationGamepad,RIGHT_BUMPER.getIndex());

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

    @Override
    public EnhancedJoystickButton getLimeAutoAimButton() {
        return null;
    }

    @Override
    public EnhancedJoystickButton getNavAutoAimButton() {
        return null;
    }
//
//    @Override
//    public EnhancedJoystickButton getNavAutoAimButton(){
//        return navAutoAimButton;
//    }

    @Override
    public EnhancedJoystickButton getIntakeButton() {
        return intakeButton;
    }

    @Override
    public EnhancedJoystickButton getOuttakeButton() {
        return outtakeButton;
    }

}
