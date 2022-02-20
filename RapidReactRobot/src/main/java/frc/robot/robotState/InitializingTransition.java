package frc.robot.robotState;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;

public class InitializingTransition implements IState {

    private double timeWhenLocksAreUnengaged;
    private final double timeForLocksToUnengageSeconds = 0.25;

    @Override
    public void initialize() {
        // Load pivot reference
        godSubsystem.getClimber().zeroSensors();

        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.DISABLED);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.DISABLED);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        // Unengage climber locks
        // Wait for climber lock pneumatics to finish movement
        timeWhenLocksAreUnengaged = godSubsystem.getCurrentTime() + timeForLocksToUnengageSeconds;
        godSubsystem.getClimber().setLeftClimberLockStateDemand(true);
        godSubsystem.getClimber().setRightClimberLockStateDemand(true);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (godSubsystem.getCurrentTime() >= timeWhenLocksAreUnengaged) {
            return new Initializing();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}