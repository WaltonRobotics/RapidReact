package frc.robot.robotState;

import frc.robot.OI;
import frc.robot.robotState.scoring.AdjustingHood;
import frc.robot.robotState.scoring.Intaking;
import frc.robot.robotState.scoring.Outtaking;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.*;

import static frc.robot.OI.shootButton;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class ScoringMode implements IState {

    @Override
    public void initialize() {
        godSubsystem.setCurrentMode(Superstructure.CurrentMode.SCORING_MODE);

        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
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

        if (shootButton.get()) {
            return new AdjustingHood();
        }

        if (OI.overrideTransportConveyorButton.get()) {
            godSubsystem.getConveyor().setTransportDemand(currentRobot.getConveyorConfig().getTransportIntakeVoltage());
        } else {
            godSubsystem.getConveyor().setTransportDemand(0);
        }

        if (OI.overrideFeedConveyorButton.get()) {
            godSubsystem.getConveyor().setFeedDemand(currentRobot.getConveyorConfig().getFeedIntakeVoltage());
        } else {
            godSubsystem.getConveyor().setFeedDemand(0);
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
