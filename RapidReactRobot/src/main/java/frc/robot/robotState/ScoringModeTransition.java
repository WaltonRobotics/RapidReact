package frc.robot.robotState;

import frc.robot.config.Target;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class ScoringModeTransition implements IState {

    private final Target stowedAngle = currentRobot.getPivotTarget(Climber.ClimberPivotPosition.STOWED_ANGLE);
    private final Target stowedHeight = currentRobot.getExtensionTarget(Climber.ClimberExtensionPosition.STOWED_HEIGHT);

    @Override
    public void initialize() {
        // Unengage climber locks
        // Unengage climber disc brake
        godSubsystem.getClimber().setLeftClimberLockStateDemand(true);
        godSubsystem.getClimber().setRightClimberLockStateDemand(true);

        godSubsystem.getClimber().setClimberDiscBrakeStateDemand(true);

        // Stow climber pivot and extension
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemand(Climber.ClimberPivotPosition.STOWED_ANGLE);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_STOWED);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(Climber.ClimberExtensionPosition.STOWED_HEIGHT);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.STOWED);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        double pivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();
        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        if (stowedAngle.isWithinTolerance(pivotAngle, 50)
                && stowedHeight.isWithinTolerance(extensionHeight, 50)) {
            return new ScoringMode();
        }

        return this;
    }

    @Override
    public void finish() {
        // Engage climber locks
        godSubsystem.getClimber().setLeftClimberLockStateDemand(false);
        godSubsystem.getClimber().setRightClimberLockStateDemand(false);
    }

}
