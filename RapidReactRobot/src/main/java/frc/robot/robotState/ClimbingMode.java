package frc.robot.robotState;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import frc.robot.OI;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Superstructure;

import static frc.robot.RobotContainer.godSubsystem;

public class ClimbingMode implements IState {

    @Override
    public void initialize() {
        godSubsystem.setCurrentMode(Superstructure.CurrentMode.CLIMBING_MODE);

        godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Coast);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setPivotPositionDemand(Climber.ClimberPivotPosition.REACHING_FOR_MID_BAR_PIVOT_ANGLE);
        godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

        godSubsystem.getClimber().setExtensionNeutralMode(NeutralMode.Coast);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.AUTO);
        godSubsystem.getClimber().setExtensionPositionDemand(Climber.ClimberExtensionPosition.HOOKING_ONTO_MID_BAR_LENGTH);
        godSubsystem.getClimber().setExtensionLimits(Climber.ClimberExtensionLimits.EXTENSION_FULL_ROM);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (OI.toggleBetweenScoringAndClimbingModeButton.isRisingEdge()) {
            return new ScoringModeTransition();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
