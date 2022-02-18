package frc.robot.controller;

import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.util.buttons.EnhancedAxisButton;
import frc.robot.util.buttons.EnhancedJoystickButton;

public interface ControllerConfig {

    double getForward();
    double getStrafe();
    double getYaw();

    ExpoScale getForwardScale();
    ExpoScale getStrafeScale();
    ExpoScale getYawScale();

    EnhancedJoystickButton getResetDrivetrainButton();
    EnhancedJoystickButton getLimeAutoAimButton();
    EnhancedJoystickButton getNavAutoAimButton();
    EnhancedAxisButton getIntakeButton();
    EnhancedJoystickButton getOuttakeButton();

}
