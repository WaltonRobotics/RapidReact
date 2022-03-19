package frc.robot.robotState;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.OI;
import frc.robot.config.Target;
import frc.robot.robotState.climbing.PullUpToHookOntoMidBar;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Superstructure;

import static frc.robot.OI.*;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class ClimbingMode implements IState {

    private final Target stowedAngle = currentRobot.getPivotTarget(Climber.ClimberPivotPosition.STOWED_ANGLE);
    private final Target hookingLength = currentRobot.getExtensionTarget(Climber.ClimberExtensionPosition.LINING_UP_TO_MID_BAR_LENGTH);

    @Override
    public void initialize() {
        godSubsystem.setCurrentMode(Superstructure.CurrentMode.CLIMBING_MODE);

        godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Brake);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionNeutralMode(NeutralMode.Brake);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(Climber.ClimberExtensionPosition.LINING_UP_TO_MID_BAR_LENGTH);
        godSubsystem.getClimber().enableExtensionLowerLimit();
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);

        godSubsystem.getClimber().setPivotPositionDemand(Climber.ClimberPivotPosition.STOWED_ANGLE);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (OI.toggleBetweenScoringAndClimbingModeButton.isRisingEdge()) {
            return new ScoringModeTransition();
        }

        double pivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();
        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        if ((stowedAngle.isWithinTolerance(pivotAngle) && hookingLength.isWithinTolerance(extensionHeight)
                && midRungAdvanceButton.get())
                || (midRungAdvanceButton.get() && overrideNextClimbStateButton.isRisingEdge())) {

            godSubsystem.setSelectedRung(Superstructure.ClimbingTargetRung.MID_RUNG);
            return new PullUpToHookOntoMidBar();
        }

        if ((stowedAngle.isWithinTolerance(pivotAngle) && hookingLength.isWithinTolerance(extensionHeight)
                && highRungAdvanceButton.get())
                || (highRungAdvanceButton.get() && overrideNextClimbStateButton.isRisingEdge())) {

            godSubsystem.setSelectedRung(Superstructure.ClimbingTargetRung.HIGH_RUNG);
            return new PullUpToHookOntoMidBar();
        }

        godSubsystem.handleExtensionManualOverride();
        godSubsystem.handlePivotManualOverride();

        return this;
    }

    @Override
    public void finish() {

    }

}
