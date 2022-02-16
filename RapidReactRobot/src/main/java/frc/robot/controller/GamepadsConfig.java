package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.Constants;
import frc.robot.util.EnhancedJoystickButton;
import frc.robot.util.Gamepad;

import static frc.robot.util.Gamepad.Button.*;

public class GamepadsConfig implements ControllerConfig {

    private final Gamepad driveGamepad = new Gamepad(Constants.ControllerPorts.GamepadsConfigPorts.kDriveGamepadPort);
    private final Gamepad manipulationGamepad = new Gamepad(Constants.ControllerPorts.GamepadsConfigPorts.kManipulationGamepadPort);

    private final ExpoScale forwardScale = new ExpoScale(0.05, 0.6);
    private final ExpoScale strafeScale = new ExpoScale(0.05, 0.6);
    private final ExpoScale yawScale = new ExpoScale(0.01, 0.75);

    private final EnhancedJoystickButton resetDrivetrainButton = new EnhancedJoystickButton(driveGamepad, LEFT_BUMPER.getIndex());
    private final EnhancedJoystickButton intakeButton = new EnhancedJoystickButton(manipulationGamepad,RIGHT_TRIGGER.getIndex());
    private final EnhancedJoystickButton outtakeButton = new EnhancedJoystickButton(manipulationGamepad,RIGHT_BUMPER.getIndex());

    @Override
    public double getForward() {
        return -driveGamepad.getLeftY();
    }

    @Override
    public double getStrafe() {
        return -driveGamepad.getLeftX();
    }

    @Override
    public double getYaw() {
        return -driveGamepad.getRightX();
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

    @Override
    public EnhancedJoystickButton getIntakeButton(){
        return intakeButton;
    }

    @Override
    public EnhancedJoystickButton getOuttakeButton(){
        return outtakeButton;
    }


}
