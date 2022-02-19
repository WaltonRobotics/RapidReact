package frc.robot.robotState;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;

public class Initializing implements IState {
    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.DISABLED);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.DISABLED);

        // Unengage climber locks
        // Load pivot reference
        // Set pivot to hold current position
        // Limit pivot ROM
        // Wait for climber lock pneumatics to finish movement
        // Zero extension
        // Limit extension ROM

        godSubsystem.getClimber().zeroSensors();
        godSubsystem.getClimber().setLeftClimberLockStateDemand(true);
        godSubsystem.getClimber().setRightClimberLockStateDemand(true);

        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.ZEROING);
        godSubsystem.getClimber().setZeroed(false);
        godSubsystem.getClimber().releaseExtensionLowerLimit();
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
