package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.Constants.Climber.kDefaultPivotAcceleration;
import static frc.robot.Constants.Climber.kDefaultPivotCruiseVelocity;
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

        godSubsystem.getClimber().setPivotPositionDemand(
                Climber.ClimberPivotPosition.REACHING_FOR_HIGH_BAR_PIVOT_ANGLE);

        godSubsystem.getClimber().configPivotSmartMotion(2100, 2400);
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

        if ((angleTarget.isWithinTolerance(pivotAngle) && advanceClimbingProcessButton.get())
                || overrideNextClimbStateButton.isRisingEdge()) {
            return new InitiateHighBarClimb();
        }

        godSubsystem.handleExtensionManualOverride();
        godSubsystem.handlePivotManualOverride();

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().configPivotSmartMotion(kDefaultPivotCruiseVelocity, kDefaultPivotAcceleration);
    }

}
