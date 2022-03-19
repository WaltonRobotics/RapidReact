package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.util.buttons.EnhancedAxisButton;
import frc.robot.util.buttons.EnhancedComboButton;
import frc.robot.util.buttons.EnhancedJoystickButton;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftTrigger;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightTrigger;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.Constants.DriverPreferences.XboxConfigPorts.kDriveXboxControllerPort;
import static frc.robot.Constants.DriverPreferences.XboxConfigPorts.kManipulationXboxControllerPort;

public class OI {

    public static final XboxController driveGamepad = new XboxController(kDriveXboxControllerPort);
    public static final XboxController manipulationGamepad = new XboxController(kManipulationXboxControllerPort);

    public static final ExpoScale forwardScale = new ExpoScale(0.1, 0.6);
    public static final ExpoScale strafeScale = new ExpoScale(0.1, 0.6);
    public static final ExpoScale yawScale = new ExpoScale(0.1, 0.75);

    // Scoring mode
    public static final EnhancedAxisButton intakeButton = new EnhancedAxisButton(manipulationGamepad,
            kLeftTrigger.value, 0.5);
    public static final EnhancedJoystickButton outtakeButton = new EnhancedJoystickButton(manipulationGamepad,
            kLeftBumper.value);
    public static final EnhancedJoystickButton overrideTransportConveyorButton = new EnhancedJoystickButton(manipulationGamepad, kBack.value);
    public static final EnhancedJoystickButton overrideFeedConveyorButton = new EnhancedJoystickButton(manipulationGamepad, kStart.value);
    public static final EnhancedJoystickButton toggleLeftIntakeButton = new EnhancedJoystickButton(manipulationGamepad, EnhancedJoystickButton.POV_W);
    public static final EnhancedJoystickButton toggleRightIntakeButton = new EnhancedJoystickButton(manipulationGamepad, EnhancedJoystickButton.POV_E);
    public static final EnhancedJoystickButton toggleLimelightButton = new EnhancedJoystickButton(driveGamepad,
            kX.value);
    public static final EnhancedAxisButton shootButton = new EnhancedAxisButton(driveGamepad,
            kRightTrigger.value, 0.5);
    public static final EnhancedJoystickButton barfButton = new EnhancedJoystickButton(driveGamepad, kRightBumper.value);
    public static final EnhancedJoystickButton idleSpinUpButton = new EnhancedJoystickButton(manipulationGamepad, kRightBumper.value);
    public static final EnhancedJoystickButton aimUpperButton = new EnhancedJoystickButton(manipulationGamepad, EnhancedJoystickButton.POV_N);
    public static final EnhancedJoystickButton aimLowerButton = new EnhancedJoystickButton(manipulationGamepad, EnhancedJoystickButton.POV_S);

    // Climbing mode
    // Out-of-the-way buttons
    public static final EnhancedJoystickButton fastZeroButton = new EnhancedJoystickButton(manipulationGamepad, kLeftBumper.value);
    public static final EnhancedAxisButton
            dangerButton = new EnhancedAxisButton(manipulationGamepad, kRightTrigger.value, 0.85);
    public static final EnhancedComboButton stopClimbButton = new EnhancedComboButton(dangerButton,
            new EnhancedAxisButton(manipulationGamepad, kLeftTrigger.value, 0.85));
    public static final EnhancedComboButton overrideNextClimbStateButton = new EnhancedComboButton(dangerButton,
            new EnhancedJoystickButton(manipulationGamepad, kB.value));
    public static final EnhancedJoystickButton advanceClimbingProcessButton = new EnhancedJoystickButton(manipulationGamepad, kA.value);
    public static final EnhancedJoystickButton selectMidRungButton = new EnhancedJoystickButton(manipulationGamepad, kBack.value);
    public static final EnhancedJoystickButton selectHighRungButton = new EnhancedJoystickButton(manipulationGamepad, kStart.value);

    // Both modes
    public static final EnhancedJoystickButton resetDrivetrainButton = new EnhancedJoystickButton(driveGamepad, kLeftBumper.value);
    public static final EnhancedComboButton toggleClimberLocksButton = new EnhancedComboButton(dangerButton,
            new EnhancedJoystickButton(manipulationGamepad, kY.value));
    public static final EnhancedComboButton toggleBetweenScoringAndClimbingModeButton = new EnhancedComboButton(
            new EnhancedJoystickButton(driveGamepad, kStart.value), new EnhancedJoystickButton(driveGamepad, kBack.value));
    public static final EnhancedJoystickButton toggleFieldRelativeModeButton = new EnhancedJoystickButton(driveGamepad, EnhancedJoystickButton.POV_N);
    public static final EnhancedJoystickButton toggleRotationModeButton = new EnhancedJoystickButton(driveGamepad, EnhancedJoystickButton.POV_S);
    public static final EnhancedJoystickButton faceClosestButton = new EnhancedJoystickButton(driveGamepad, kA.value);
    public static final EnhancedJoystickButton faceClimbButton = new EnhancedJoystickButton(driveGamepad, kB.value);
    public static final EnhancedAxisButton trackTargetButton = new EnhancedAxisButton(driveGamepad,
            kLeftTrigger.value, 0.5);

}
