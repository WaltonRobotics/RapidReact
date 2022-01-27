package frc.robot.controller;

import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.Constants;
import frc.robot.util.Gamepad;

public class GamepadsConfig implements ControllerConfig {

    private final Gamepad driveGamepad = new Gamepad(Constants.ControllerPorts.GamepadsConfigPorts.kDriveGamepadPort);
    private final Gamepad manipulationGamepad = new Gamepad(Constants.ControllerPorts.GamepadsConfigPorts.kManipulationGamepadPort);

    private final ExpoScale forwardScale = new ExpoScale(0.05, 0.6);
    private final ExpoScale strafeScale = new ExpoScale(0.05, 0.6);
    private final ExpoScale yawScale = new ExpoScale(0.01, 0.75);

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

}
