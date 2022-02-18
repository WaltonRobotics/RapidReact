package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.util.buttons.EnhancedAxisButton;
import frc.robot.util.buttons.EnhancedComboButton;
import frc.robot.util.buttons.EnhancedJoystickButton;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftTrigger;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightTrigger;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static edu.wpi.first.wpilibj.XboxController.Button.kLeftBumper;
import static frc.robot.Constants.ControllerPorts.XboxConfigPorts.kDriveXboxControllerPort;
import static frc.robot.Constants.ControllerPorts.XboxConfigPorts.kManipulationXboxControllerPort;

public class OI {
    
    public static XboxController driveGamepad = new XboxController(kDriveXboxControllerPort);
    public static XboxController manipulationGamepad = new XboxController(kManipulationXboxControllerPort);

    public static final ExpoScale forwardScale = new ExpoScale(0.1, 0.6);
    public static final ExpoScale strafeScale = new ExpoScale(0.1, 0.6);
    public static final ExpoScale yawScale = new ExpoScale(0.1, 0.75);

    public static final EnhancedJoystickButton limeAutoAimButton = new EnhancedJoystickButton(driveGamepad, kRightBumper.value);
    public static final EnhancedJoystickButton navAutoAimButton = new EnhancedJoystickButton(driveGamepad, kA.value);

    // Scoring mode
    public static final EnhancedAxisButton intakeButton = new EnhancedAxisButton(manipulationGamepad,
            kLeftTrigger.value, 0.5);
    public static final EnhancedJoystickButton outtakeButton = new EnhancedJoystickButton(manipulationGamepad,
            kLeftBumper.value);
    public static EnhancedJoystickButton overrideTransportConveyorButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Left middle
    public static EnhancedJoystickButton overrideFeedConveyorButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Right middle
    public static EnhancedJoystickButton toggleLeftIntakeButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Left dpad
    public static EnhancedJoystickButton toggleRightIntakeButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Right dpad
    public static EnhancedAxisButton shootButtonButton = new EnhancedAxisButton(manipulationGamepad,
            kRightTrigger.value, 0.5); // RT
    public static EnhancedJoystickButton barfButtonButton = new EnhancedJoystickButton(manipulationGamepad, 0); // RB

    // Climbing mode
    // Out-of-the-way buttons
    public static EnhancedJoystickButton dangerButton = new EnhancedJoystickButton(manipulationGamepad, 0); // RT
    public static EnhancedJoystickButton stopClimbButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Center

    public static EnhancedJoystickButton toggleClimberLocksButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Y
    public static EnhancedJoystickButton advanceClimbingProcessButton = new EnhancedJoystickButton(manipulationGamepad, 0); // A

    // Both modes
    public static final EnhancedJoystickButton resetDrivetrainButton = new EnhancedJoystickButton(driveGamepad, kLeftBumper.value);
    public static final EnhancedComboButton toggleBetweenScoringAndClimbingModeButton = new EnhancedComboButton();

}
