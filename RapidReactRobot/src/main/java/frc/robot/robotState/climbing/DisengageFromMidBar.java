package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.OI.*;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class DisengageFromMidBar implements IState {

    private final Target heightTarget =
            currentRobot.getExtensionTarget(Climber.ClimberExtensionPosition.LENGTH_TO_DISENGAGE_FROM_MID_BAR);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(
                Climber.ClimberExtensionPosition.LENGTH_TO_DISENGAGE_FROM_MID_BAR);
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

        if ((heightTarget.isWithinTolerance(extensionHeight) && midRungAdvanceButton.get())
                || overrideNextClimbStateButton.isRisingEdge()) {
            return new RotatePivotForHighBar();
        }

        godSubsystem.handleExtensionManualOverride();
        godSubsystem.handlePivotManualOverride();

        return this;
    }

    @Override
    public void finish() {

    }

}
