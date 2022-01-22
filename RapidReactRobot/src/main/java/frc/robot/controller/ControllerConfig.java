package frc.robot.controller;

import frc.robot.util.Gamepad;

public interface ControllerConfig {

    double getForward();
    double getStrafe();
    double getYaw();

    Gamepad getManipulatorGamepad();

}
