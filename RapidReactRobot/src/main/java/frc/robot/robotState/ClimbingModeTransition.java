package frc.robot.robotState;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;

public class ClimbingModeTransition implements IState {

    private double timeWhenLocksAreEngaged;
    private final double timeForLocksToEngageSeconds = 0.25;

    @Override
    public void initialize() {
        // Unengage climber locks
        timeWhenLocksAreEngaged = godSubsystem.getCurrentTime() + timeForLocksToEngageSeconds;
        godSubsystem.getClimber().setLeftClimberLockStateDemand(true);
        godSubsystem.getClimber().setRightClimberLockStateDemand(true);

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

        if (godSubsystem.getCurrentTime() >= timeWhenLocksAreEngaged) {
            return new ClimbingMode();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
