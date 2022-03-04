package frc.robot.robotState;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
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

        godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Coast);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionNeutralMode(NeutralMode.Coast);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(Climber.ClimberExtensionPosition.LINING_UP_TO_MID_BAR_LENGTH);
        godSubsystem.getClimber().enableExtensionLowerLimit();
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (OI.toggleBetweenScoringAndClimbingModeButton.isRisingEdge()) {
            return new ScoringModeTransition();
        }

        double pivotAngle = godSubsystem.getClimber().getPivotAbsoluteEncoderPositionNU();
        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        if ((stowedAngle.isWithinTolerance(pivotAngle) && hookingLength.isWithinTolerance(extensionHeight)
                && advanceClimbingProcessButton.get()) || overrideNextClimbStateButton.isRisingEdge()) {
            return new PullUpToHookOntoMidBar();
        }

        Rotation2d currentRobotPitch = godSubsystem.getDrivetrain().getPitch();
        double ff = godSubsystem.getClimber().getCalculatedFeedForward(currentRobotPitch);

        // Pivot arm is off the hook and needs a feedforward
        godSubsystem.getClimber().setPivotPositionDemand(Climber.ClimberPivotPosition.STOWED_ANGLE);

        godSubsystem.handleExtensionManualOverride();
        godSubsystem.handlePivotManualOverride();

        return this;
    }

    @Override
    public void finish() {

    }

}
