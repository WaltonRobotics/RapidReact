package frc.robot.robotState.scoring;

import frc.robot.commands.DriveCommand;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringModeTransition;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Shooter.kSpinDownTimeSeconds;
import static frc.robot.OI.intakeButton;
import static frc.robot.OI.outtakeButton;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SHOOTING_SLOT;

public class SpinningDown implements IState {

    private final Shooter shooter = godSubsystem.getShooter();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    private double timeout;

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.OPEN_LOOP);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        shooter.setSelectedProfileSlot(SHOOTING_SLOT);
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        conveyor.setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);

        timeout = godSubsystem.getCurrentTime() + kSpinDownTimeSeconds;

        DriveCommand.setIsEnabled(true);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (godSubsystem.getCurrentTime() >= timeout) {
            return new ScoringModeTransition();
        }

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        if (intakeButton.get() || (godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToIntake())) {
            godSubsystem.handleIntaking();
        } else if (outtakeButton.get()) {
            godSubsystem.handleOuttaking();
        }

        conveyor.setTransportDemand(conveyor.getConfig().getTransportShootPercentOutput());
        conveyor.setFeedDemand(conveyor.getConfig().getFeedShootPercentOutput());

        return this;
    }

    @Override
    public void finish() {
        godSubsystem.getConveyor().setTransportDemand(0.0);
        godSubsystem.getConveyor().setFeedDemand(0.0);
    }

}
