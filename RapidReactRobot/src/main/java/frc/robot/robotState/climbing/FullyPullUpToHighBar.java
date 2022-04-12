package frc.robot.robotState.climbing;

import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.OI.stopClimbButton;
import static frc.robot.RobotContainer.godSubsystem;

public class FullyPullUpToHighBar implements IState {

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_PULL_UP_TO_HIGH_BAR);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.OPEN_LOOP);
        godSubsystem.getClimber().releaseExtensionLowerLimit();
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (stopClimbButton.isRisingEdge()) {
            return new FinalizeClimb();
        }

        if (godSubsystem.getClimber().isLeftExtensionLowerLimitClosed() ||
                godSubsystem.getClimber().isRightExtensionLowerLimitClosed()) {
            return new TransferHighBarFromPivotToFixed();
        }

        godSubsystem.getClimber().setExtensionPercentOutputDemand(-0.2);

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPercentOutputDemand(0);
    }
}
