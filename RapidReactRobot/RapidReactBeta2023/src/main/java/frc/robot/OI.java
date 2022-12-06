package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;

import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.Constants.DriverPreferences.XboxConfigPorts.kDriveXboxControllerPort;

public class OI {

    public static final XboxController driveGamepad = new XboxController(kDriveXboxControllerPort);

    public static final ExpoScale forwardScale = new ExpoScale(0.1, 0.6);
    public static final ExpoScale strafeScale = new ExpoScale(0.1, 0.6);
    public static final ExpoScale yawScale = new ExpoScale(0.1, 0.75);

    public static final Trigger toggleLimelightButton = new JoystickButton(driveGamepad, XboxController.Button.kY.value);
    public static final Trigger resetDrivetrainButton = new JoystickButton(driveGamepad, kLeftBumper.value);


}
