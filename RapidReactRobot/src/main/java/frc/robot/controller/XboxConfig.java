package frc.robot.controller;

import edu.wpi.first.wpilibj.XboxController;
import frc.lib.strykeforce.thirdcoast.util.ExpoScale;
import frc.robot.util.buttons.EnhancedAxisButton;
import frc.robot.util.buttons.EnhancedJoystickButton;
import frc.robot.util.buttons.EnhancedComboButton;

import static edu.wpi.first.wpilibj.XboxController.Axis.kLeftTrigger;
import static edu.wpi.first.wpilibj.XboxController.Axis.kRightTrigger;
import static edu.wpi.first.wpilibj.XboxController.Button.*;
import static frc.robot.Constants.ControllerPorts.XboxConfigPorts.kDriveXboxControllerPort;
import static frc.robot.Constants.ControllerPorts.XboxConfigPorts.kManipulationXboxControllerPort;

public class XboxConfig implements ControllerConfig{
    public static XboxController driveGamepad = new XboxController(kDriveXboxControllerPort);
    public static XboxController manipulationGamepad = new XboxController(kManipulationXboxControllerPort);

    private final ExpoScale forwardScale = new ExpoScale(0.1, 0.6);
    private final ExpoScale strafeScale = new ExpoScale(0.1, 0.6);
    private final ExpoScale yawScale = new ExpoScale(0.1, 0.75);

    private final EnhancedJoystickButton limeAutoAimButton = new EnhancedJoystickButton(driveGamepad, kRightBumper.value);
    private final EnhancedJoystickButton navAutoAimButton = new EnhancedJoystickButton(driveGamepad, kA.value);

    // Scoring mode
    private final EnhancedAxisButton intakeButton = new EnhancedAxisButton(manipulationGamepad,
            kLeftTrigger.value, 0.5);
    private final EnhancedJoystickButton outtakeButton = new EnhancedJoystickButton(manipulationGamepad,
            kLeftBumper.value);
    private EnhancedJoystickButton overrideTransportConveyorButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Left middle
    private EnhancedJoystickButton overrideFeedConveyorButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Right middle
    private EnhancedJoystickButton toggleLeftIntakeButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Left dpad
    private EnhancedJoystickButton toggleRightIntakeButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Right dpad
    private EnhancedAxisButton shootButtonButton = new EnhancedAxisButton(manipulationGamepad,
            kRightTrigger.value, 0.5); // RT
    private EnhancedJoystickButton barfButtonButton = new EnhancedJoystickButton(manipulationGamepad, 0); // RB

    // Climbing mode
    // Out-of-the-way buttons
    private EnhancedJoystickButton dangerButton = new EnhancedJoystickButton(manipulationGamepad, 0); // RT
    private EnhancedJoystickButton stopClimbButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Center

    private EnhancedJoystickButton toggleClimberLocksButton = new EnhancedJoystickButton(manipulationGamepad, 0); // Y
    private EnhancedJoystickButton advanceClimbingProcessButton = new EnhancedJoystickButton(manipulationGamepad, 0); // A

    // Both modes
    private final EnhancedJoystickButton resetDrivetrainButton = new EnhancedJoystickButton(driveGamepad, kLeftBumper.value);
    private final EnhancedComboButton toggleBetweenScoringAndClimbingModeButton = new EnhancedComboButton();

    @Override
    public double getForward() {
        return -driveGamepad.getLeftY();
    }

    @Override
    public double getStrafe() {
        return -driveGamepad.getLeftX();
    }

    @Override
    public double getYaw() {
        return -driveGamepad.getRightX();
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

    @Override
    public EnhancedJoystickButton getResetDrivetrainButton() {
        return resetDrivetrainButton;
    }

    @Override
    public EnhancedJoystickButton getLimeAutoAimButton() {
        return limeAutoAimButton;
    }

    public EnhancedJoystickButton getNavAutoAimButton(){
        return navAutoAimButton;
    }

    @Override
    public EnhancedAxisButton getIntakeButton() {
        return intakeButton;
    }

    @Override
    public EnhancedJoystickButton getOuttakeButton() {
        return outtakeButton;
    }

}
