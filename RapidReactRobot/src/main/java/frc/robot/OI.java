package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
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

    public static final XboxController driveGamepad = new XboxController(kDriveXboxControllerPort);
    public static final XboxController manipulationGamepad = new XboxController(kManipulationXboxControllerPort);

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
    public static final EnhancedJoystickButton overrideTransportConveyorButton = new EnhancedJoystickButton(manipulationGamepad, kBack.value);
    public static final EnhancedJoystickButton overrideFeedConveyorButton = new EnhancedJoystickButton(manipulationGamepad, kStart.value);
    public static final EnhancedJoystickButton toggleLeftIntakeButton = new EnhancedJoystickButton(manipulationGamepad, EnhancedJoystickButton.POV_W);
    public static final EnhancedJoystickButton toggleRightIntakeButton = new EnhancedJoystickButton(manipulationGamepad, EnhancedJoystickButton.POV_E);
    public static final EnhancedAxisButton shootButtonButton = new EnhancedAxisButton(manipulationGamepad,
            kRightTrigger.value, 0.5);
    public static final EnhancedJoystickButton barfButtonButton = new EnhancedJoystickButton(manipulationGamepad, kRightBumper.value);

    // Climbing mode
    // Out-of-the-way buttons
    public static final EnhancedAxisButton dangerButton = new EnhancedAxisButton(manipulationGamepad, kRightTrigger.value, 0.5);
    public static final EnhancedComboButton stopClimbButton = new EnhancedComboButton(dangerButton,
            new EnhancedAxisButton(manipulationGamepad, kLeftTrigger.value, 0.5));

    private static final EnhancedJoystickButton yButton = new EnhancedJoystickButton(manipulationGamepad, kY.value);

    public static final EnhancedComboButton toggleClimberLocksButton = new EnhancedComboButton(dangerButton, yButton);
    public static final EnhancedJoystickButton advanceClimbingProcessButton = new EnhancedJoystickButton(manipulationGamepad, kA.value);

    // Both modes
    public static final EnhancedJoystickButton resetDrivetrainButton = new EnhancedJoystickButton(driveGamepad, kLeftBumper.value);
    public static final EnhancedComboButton toggleBetweenScoringAndClimbingModeButton = new EnhancedComboButton(
            new EnhancedJoystickButton(manipulationGamepad,kX.value), yButton);

}
