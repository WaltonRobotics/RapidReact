package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.OI.*;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class TransferMidBarFromPivotToFixed implements IState {

    private final Target stowedAngle = currentRobot.getPivotTarget(Climber.ClimberPivotPosition.PIVOT_BACK_TO_TRANSFER);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemand(Climber.ClimberPivotPosition.PIVOT_BACK_TO_TRANSFER);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPercentOutputDemand(0);
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

        double pivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();

        if ((stowedAngle.isWithinTolerance(pivotAngle) && advanceClimbingProcessButton.get())
                || overrideNextClimbStateButton.isRisingEdge()) {
            return new DisengageFromMidBar();
        }

        godSubsystem.handlePivotManualOverride();

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().enableExtensionLowerLimit();
    }

}
