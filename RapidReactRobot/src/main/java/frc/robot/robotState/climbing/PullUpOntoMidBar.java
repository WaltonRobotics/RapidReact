package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.Climber.kTransferPercentOutput;
import static frc.robot.OI.*;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class PullUpOntoMidBar implements IState {

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_PULL_UP_TO_MID_BAR);

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
            return new TransferMidBarFromPivotToFixed();
        }

        godSubsystem.getClimber().setExtensionPercentOutputDemand(kTransferPercentOutput);

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPercentOutputDemand(0);
    }

}
