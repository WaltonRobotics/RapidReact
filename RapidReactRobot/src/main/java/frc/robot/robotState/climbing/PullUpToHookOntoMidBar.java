package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.OI.*;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class PullUpToHookOntoMidBar implements IState {

    private final Target pullUpLength = currentRobot.getExtensionTarget(
            Climber.ClimberExtensionPosition.PULL_UP_TO_HOOK_ONTO_MID_BAR_LENGTH);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemand(Climber.ClimberPivotPosition.STOWED_ANGLE);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_STOWED);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(
                Climber.ClimberExtensionPosition.PULL_UP_TO_HOOK_ONTO_MID_BAR_LENGTH);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
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

        if ((pullUpLength.isWithinTolerance(extensionHeight) && advanceClimbingProcessButton.get())
                || overrideNextClimbStateButton.isRisingEdge()) {
            return new PositionFixedArmForMidBarTransfer();
        }

        godSubsystem.handleExtensionManualOverride();

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.MID_BAR_POSITION_FIXED_ARM);
    }

}
