package frc.robot.robotState;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;

public class ClimbingModeTransition implements IState {

    private final double timeForLocksToUnengageSeconds = 0.25;
    private double timeWhenLocksAreUnengaged;

    @Override
    public void initialize() {
        // Reload pivot reference
        godSubsystem.getClimber().zeroSensors();

        // Unengage climber locks
        // Unengage climber disc brake
        timeWhenLocksAreUnengaged = godSubsystem.getCurrentTime() + timeForLocksToUnengageSeconds;
        godSubsystem.getClimber().setLeftClimberLockStateDemand(true);
        godSubsystem.getClimber().setRightClimberLockStateDemand(true);

        godSubsystem.getClimber().setClimberDiscBrakeStateDemand(true);

        // Pull up both intakes
        godSubsystem.getIntake().setLeftIntakeDeployStateDemand(false);
        godSubsystem.getIntake().setRightIntakeDeployStateDemand(false);

        // Disable all subsystems
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.DISABLED);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.DISABLED);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (godSubsystem.getCurrentTime() >= timeWhenLocksAreUnengaged) {
            return new ClimbingModeZeroing();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
