package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.OI.barfButton;
import static frc.robot.OI.shootButton;
import static frc.robot.RobotContainer.godSubsystem;

public class Intaking implements IState {

    private final Intake intake = godSubsystem.getIntake();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    @Override
    public void initialize() {
        intake.setIntakeControlState(Intake.IntakeControlState.OPEN_LOOP);
        conveyor.setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.DISABLED);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.intakeButton.get() && !((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToIntake()))) {
            return new ScoringMode();
        }

        if (shootButton.isRisingEdge() || barfButton.isRisingEdge()
                || (godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToShoot())
                || (godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToAlignAndShoot())) {
            return new AdjustingHood();
        }

        godSubsystem.handleIntakingWithConveyor();

        return this;
    }

    @Override
    public void finish() {

    }

}
