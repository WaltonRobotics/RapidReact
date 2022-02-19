package frc.robot.robotState.climbing;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;

import static frc.robot.RobotContainer.godSubsystem;

public class DisengageFromMidBar implements IState {

    @Override
    public void initialize() {
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(
                Climber.ClimberExtensionPosition.LENGTH_TO_DISENGAGE_FROM_MID_BAR);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

    @Override
    public IState execute() {
        return null;
    }

    @Override
    public void finish() {

    }

}
