package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.OI.*;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class FullyPullUpToHighBar implements IState {

    private Target heightTarget = currentRobot.getExtensionTarget(Climber.ClimberExtensionPosition.STOWED_HEIGHT);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_PULL_UP_TO_TRANSFER_HIGH_BAR);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(Climber.ClimberExtensionPosition.STOWED_HEIGHT);
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

        if ((heightTarget.isWithinTolerance(extensionHeight, 50)&& advanceClimbingProcessButton.get())
                || overrideNextClimbStateButton.isRisingEdge()) {
                return new TransferHighBarFromPivotToFixed();
        }

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.STOWED);
    }

}
