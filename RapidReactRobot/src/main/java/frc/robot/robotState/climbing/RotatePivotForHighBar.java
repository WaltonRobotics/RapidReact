package frc.robot.robotState.climbing;

import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.OI.*;
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
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (stopClimbButton.isRisingEdge()) {
            return new FinalizeClimb();
        }

        double pivotAngle = godSubsystem.getClimber().getPivotAbsoluteEncoderPositionNU();

        if ((angleTarget.isWithinTolerance(pivotAngle) && advanceClimbingProcessButton.get())
                || overrideNextClimbStateButton.isRisingEdge()) {
            return new InitiateHighBarClimb();
        }

        Rotation2d currentRobotPitch = godSubsystem.getDrivetrain().getPitch();
        double ff = godSubsystem.getClimber().getCalculatedFeedForward(currentRobotPitch);

        godSubsystem.getClimber().setPivotPositionDemand(
                Climber.ClimberPivotPosition.REACHING_FOR_HIGH_BAR_PIVOT_ANGLE, ff);

        godSubsystem.handleExtensionManualOverride();

        return this;
    }

    @Override
    public void finish() {

    }

}
