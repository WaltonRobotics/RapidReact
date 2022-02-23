package frc.robot.robotState.scoring;

import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Shooter.kSpinDownTimeSeconds;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SHOOTING_SLOT;

public class SpinningDown implements IState {

    private final Shooter shooter = godSubsystem.getShooter();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    private double timeout;

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        shooter.setSelectedProfileSlot(SHOOTING_SLOT);
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        conveyor.setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);

        timeout = godSubsystem.getCurrentTime() + kSpinDownTimeSeconds;
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (godSubsystem.getCurrentTime() >= timeout) {
            return new ScoringMode();
        }

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        conveyor.setTransportDemand(conveyor.getConfig().getTransportShootVoltage());
        conveyor.setFeedDemand(conveyor.getConfig().getFeedShootVoltage());

        return this;
    }

    @Override
    public void finish() {

    }

}
