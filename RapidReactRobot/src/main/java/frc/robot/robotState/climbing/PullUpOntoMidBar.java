package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class PullUpOntoMidBar implements IState {

    private final Target targetLength = currentRobot.getExtensionTarget(
            Climber.ClimberExtensionPosition.STOWED_HEIGHT);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_PULL_UP_TO_MID_BAR);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(Climber.ClimberExtensionPosition.STOWED_HEIGHT);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

    @Override
    public IState execute() {
        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        if (targetLength.isWithinTolerance(extensionHeight, 50)) {
            return new TransferMidBarFromPivotToFixed();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
