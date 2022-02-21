package frc.robot.robotState;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;

public class InitializingTransition implements IState {

    private final double timeForLocksToUnengageSeconds = 0.25;
    private double timeWhenLocksAreUnengaged;

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
        // Unengage climber disc brake
        // Wait for climber pneumatics to finish movement
        timeWhenLocksAreUnengaged = godSubsystem.getCurrentTime() + timeForLocksToUnengageSeconds;
        godSubsystem.getClimber().setLeftClimberLockStateDemand(true);
        godSubsystem.getClimber().setRightClimberLockStateDemand(true);

        godSubsystem.getClimber().setClimberDiscBrakeStateDemand(true);
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
