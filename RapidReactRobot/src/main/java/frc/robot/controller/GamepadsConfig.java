package frc.robot.controller;

import frc.robot.Constants;
import frc.robot.util.Gamepad;

public class GamepadsConfig implements ControllerConfig {

    private final Gamepad driveGamepad = new Gamepad(Constants.ControllerPorts.GamepadsConfigPorts.kDriveGamepadPort);
    private final Gamepad manipulationGamepad = new Gamepad(Constants.ControllerPorts.GamepadsConfigPorts.kManipulationGamepadPort);

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

}
