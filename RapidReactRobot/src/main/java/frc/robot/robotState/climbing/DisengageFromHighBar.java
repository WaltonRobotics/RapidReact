package frc.robot.robotState.climbing;

import frc.robot.config.Target;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class DisengageFromHighBar implements IState {

    private final Target heightTarget =
            currentRobot.getExtensionTarget(Climber.ClimberExtensionPosition.LENGTH_TO_DISENGAGE_FROM_HIGH_BAR);

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(
                Climber.ClimberExtensionPosition.LENGTH_TO_DISENGAGE_FROM_HIGH_BAR);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

    @Override
    public IState execute() {
        double extensionHeight = godSubsystem.getClimber().getExtensionIntegratedEncoderPosition();

        if (heightTarget.isWithinTolerance(extensionHeight, 50)) {
            return new RotatePivotForTraversalBar();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
