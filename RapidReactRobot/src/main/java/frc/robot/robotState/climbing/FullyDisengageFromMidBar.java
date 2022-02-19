package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class FullyDisengageFromMidBar implements IState {

    private final Target angleTarget = currentRobot.getPivotTarget(
            Climber.ClimberPivotPosition.REACHING_FOR_HIGH_BAR_PIVOT_ANGLE);

    private final Target heightTarget = currentRobot.getExtensionTarget(
            Climber.ClimberExtensionPosition.LENGTH_TO_FULLY_DISENGAGE_FROM_MID_BAR);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(
                Climber.ClimberExtensionPosition.LENGTH_TO_FULLY_DISENGAGE_FROM_MID_BAR);

        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

    @Override
    public IState execute() {
        double ff = godSubsystem.getClimber().getCalculatedFeedForward();

        godSubsystem.getClimber().setPivotPositionDemand(
                Climber.ClimberPivotPosition.REACHING_FOR_HIGH_BAR_PIVOT_ANGLE, ff);

        double pivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();
        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        if (angleTarget.isWithinTolerance(pivotAngle) && heightTarget.isWithinTolerance(extensionHeight)) {
            return new InitiateHighBarClimb();
        }

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_PULL_UP_TO_HIGH_BAR);
    }

}
