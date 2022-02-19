package frc.robot.robotState;

import frc.robot.OI;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.*;

import static frc.robot.RobotContainer.godSubsystem;

public class ScoringMode implements IState {

    @Override
    public void initialize() {
        godSubsystem.setCurrentMode(Superstructure.CurrentMode.SCORING_MODE);

        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.VOLTAGE);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.DISABLED);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (OI.toggleBetweenScoringAndClimbingModeButton.isRisingEdge()) {
            return new ClimbingModeTransition();
        }

        if (OI.intakeButton.get()) {
            return new Intaking();
        }

        if (OI.outtakeButton.get()) {
            return new Outtaking();
        }

        if (OI.toggleLeftIntakeButton.isRisingEdge()) {
            godSubsystem.getIntake().toggleLeftIntakeDeployStateDemand();
        }

        if (OI.toggleRightIntakeButton.isRisingEdge()) {
            godSubsystem.getIntake().toggleRightIntakeDeployStateDemand();
        }

        if (OI.overrideTransportConveyorButton.getAsBoolean()) {
            godSubsystem.getConveyor().setTransportDemand(8.0);
        } else {
            godSubsystem.getConveyor().setTransportDemand(0);
        }

        if (OI.overrideFeedConveyorButton.getAsBoolean()) {
            godSubsystem.getConveyor().setFeedDemand(8.0);
        } else {
            godSubsystem.getConveyor().setFeedDemand(0.0);
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
