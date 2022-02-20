package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class PositionFixedArmForTransfer implements IState {

    private final Target angleTarget =
            currentRobot.getPivotTarget(Climber.ClimberPivotPosition.ANGLE_TO_POSITION_FIXED_ARM_FOR_HIGH_BAR_TRANSFER);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemand(
                Climber.ClimberPivotPosition.ANGLE_TO_POSITION_FIXED_ARM_FOR_HIGH_BAR_TRANSFER);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.HIGH_BAR_TRANSFER_TO_FIXED_ARM);
    }

    @Override
    public IState execute() {
        double pivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();

        if (angleTarget.isWithinTolerance(pivotAngle, 50)) {
            return new FullyPullUpToHighBar();
        }

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_PULL_UP_TO_TRANSFER_HIGH_BAR);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

}
