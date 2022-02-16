package frc.robot.controller;

import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.util.EnhancedJoystickButton;

public interface ControllerConfig {

    double getForward();
    double getStrafe();
    double getYaw();

    ExpoScale getForwardScale();
    ExpoScale getStrafeScale();
    ExpoScale getYawScale();

    EnhancedJoystickButton getResetDrivetrainButton();
    EnhancedJoystickButton getIntakeButton();
    EnhancedJoystickButton getOuttakeButton();

}
