package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class PositionFixedArmForMidBarTransfer implements IState {

    private final Target angleTarget =
            currentRobot.getPivotTarget(Climber.ClimberPivotPosition.ANGLE_HOOK_THETA_FOR_MID_BAR);

    // This state moves the fixed arm CCW so that it does not get in the way of the pivot arm retracting
    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemand(Climber.ClimberPivotPosition.ANGLE_HOOK_THETA_FOR_MID_BAR);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

    @Override
    public IState execute() {
        double pivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();

        if (angleTarget.isWithinTolerance(pivotAngle, 50)) {
            return new PullUpOntoMidBar();
        }

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_PULL_UP_TO_MID_BAR);
    }

}
