package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.OI.overrideNextClimbStateButton;
import static frc.robot.OI.stopClimbButton;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Climber.ClimberPivotPosition.ANGLE_HOOK_THETA_FOR_MID_BAR;

public class HighBarClimbPullUpToMidBar implements IState {

    private final Target pullUpLength = currentRobot.getExtensionTarget(
            Climber.ClimberExtensionPosition.CLOSE_IN_TO_ZERO_LENGTH);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemand(ANGLE_HOOK_THETA_FOR_MID_BAR);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_PULL_UP_TO_MID_BAR);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(
                Climber.ClimberExtensionPosition.CLOSE_IN_TO_ZERO_LENGTH);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);

        godSubsystem.getClimber().configExtensionSmartMotion(20000, 40000);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (stopClimbButton.isRisingEdge()) {
            return new FinalizeClimb();
        }

        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        if ((pullUpLength.isWithinTolerance(extensionHeight))
                || overrideNextClimbStateButton.isRisingEdge()) {
            return new PullUpOntoMidBar();
        }

        godSubsystem.handleExtensionManualOverride();

        return this;
    }

    @Override
    public void finish() {

    }

}
