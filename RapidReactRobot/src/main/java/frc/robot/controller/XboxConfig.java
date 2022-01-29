package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;

public class XboxConfig implements ControllerConfig{
    public static XboxController gamepad = new XboxController(0);
    public static XboxController manipulationGamepad = new XboxController(1);

    private final ExpoScale forwardScale = new ExpoScale(0.05, 0.6);
    private final ExpoScale strafeScale = new ExpoScale(0.05, 0.6);
    private final ExpoScale yawScale = new ExpoScale(0.01, 0.75);


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
}
