package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class RotatePivotForHighBar implements IState {

    private final Target angleTarget = currentRobot.getPivotTarget(
            Climber.ClimberPivotPosition.REACHING_FOR_HIGH_BAR_PIVOT_ANGLE);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

    @Override
    public IState execute() {
        double ff = godSubsystem.getClimber().getCalculatedFeedForward();

        godSubsystem.getClimber().setPivotPositionDemand(
                Climber.ClimberPivotPosition.REACHING_FOR_HIGH_BAR_PIVOT_ANGLE, ff);

        double pivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();

        if (angleTarget.isWithinTolerance(pivotAngle)) {
            return new InitiateHighBarClimb();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}