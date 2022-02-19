package frc.robot.robotState;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;

public class InitializingTransition implements IState {

    private double timeWhenLocksAreEngaged;
    private final double timeForLocksToEngageSeconds = 0.25;

    @Override
    public void initialize() {
        // Load pivot reference
        godSubsystem.getClimber().zeroSensors();

        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.DISABLED);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.DISABLED);

        // Unengage climber locks
        // Wait for climber lock pneumatics to finish movement
        timeWhenLocksAreEngaged = godSubsystem.getCurrentTime() + timeForLocksToEngageSeconds;
        godSubsystem.getClimber().setLeftClimberLockStateDemand(true);
        godSubsystem.getClimber().setRightClimberLockStateDemand(true);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (godSubsystem.getCurrentTime() >= timeWhenLocksAreEngaged) {
            return new Initializing();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
