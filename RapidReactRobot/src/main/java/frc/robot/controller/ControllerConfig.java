package frc.robot.controller;

import frc.lib.strykeforce.thirdcoast.util.ExpoScale;

public interface ControllerConfig {

    double getForward();
    double getStrafe();
    double getYaw();

    ExpoScale getForwardScale();
    ExpoScale getStrafeScale();
    ExpoScale getYawScale();

}
