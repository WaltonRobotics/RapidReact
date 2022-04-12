package frc.robot.robotState;

import frc.robot.commands.DriveCommand;
import frc.robot.config.Target;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.*;

import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class TakeControl implements IState {

    private final Target angleTarget = currentRobot.getPivotTarget(Climber.ClimberPivotPosition.STOWED_ANGLE);

    @Override
    public void initialize() {
        // Enable drive
        DriveCommand.setIsEnabled(true);

        // Load swerve zeros
        godSubsystem.getDrivetrain().reloadAzimuthZeros();

        // Load pivot reference
        godSubsystem.getClimber().zeroSensors();

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

        double pivotAngle = godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU();

        if (
//                angleTarget.isWithinTolerance(pivotAngle, 100) &&
                godSubsystem.getCurrentMode() == Superstructure.CurrentMode.SCORING_MODE
                        || godSubsystem.isInAuton()) {
            return new ScoringModeTransition();
        } else if (godSubsystem.getCurrentMode() == Superstructure.CurrentMode.CLIMBING_MODE) {
            return new ClimbingModeTransition();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
