package frc.robot.controller;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.Constants;
import frc.robot.util.Gamepad;

public class JoysticksConfig implements ControllerConfig {

    private Joystick leftJoystick = new Joystick(Constants.ControllerPorts.JoysticksConfigPorts.kLeftJoystickPort);
    private Joystick rightJoystick = new Joystick(Constants.ControllerPorts.JoysticksConfigPorts.kRightJoystickPort);
    private Gamepad manipulationGamepad = new Gamepad(Constants.ControllerPorts.JoysticksConfigPorts.kManipulationGamepadPort);

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
