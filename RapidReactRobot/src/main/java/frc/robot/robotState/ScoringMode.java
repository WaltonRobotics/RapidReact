package frc.robot.robotState;

import frc.robot.OI;
import frc.robot.robotState.scoring.AdjustingHood;
import frc.robot.robotState.scoring.Intaking;
import frc.robot.robotState.scoring.Outtaking;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.*;

import static frc.robot.OI.barfButton;
import static frc.robot.OI.shootButton;
import static frc.robot.RobotContainer.godSubsystem;

public class ScoringMode implements IState {

    @Override
    public void initialize() {
        godSubsystem.setCurrentMode(Superstructure.CurrentMode.SCORING_MODE);

        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.VELOCITY);
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

        if (OI.intakeButton.get()
                || (godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToIntake())) {
            return new Intaking();
        }

        if (OI.outtakeButton.get()) {
            return new Outtaking();
        }

        if (shootButton.isRisingEdge() || barfButton.isRisingEdge()
                || (godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToShoot())
                || (godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToAlignAndShoot())) {
            return new AdjustingHood();
        }

        godSubsystem.handleTrackTarget();

        godSubsystem.handleTransportConveyorManualOverride();
        godSubsystem.handleFeedConveyorManualOverride();

        return this;
    }

    @Override
    public void finish() {

    }

}
