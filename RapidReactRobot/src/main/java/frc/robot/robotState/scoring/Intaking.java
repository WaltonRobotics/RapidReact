package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.currentRobot;
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

        if (!OI.intakeButton.get()) {
            return new ScoringMode();
        }

        if (intake.isLeftIntakeDeployed()) {
            intake.setLeftIntakeDemand(currentRobot.getIntakeConfig().getLeftIntakeVoltage());
        } else {
            intake.setLeftIntakeDemand(0);
        }

        if (intake.isRightIntakeDeployed()) {
            intake.setLeftIntakeDemand(currentRobot.getIntakeConfig().getRightIntakeVoltage());
        } else {
            intake.setRightIntakeDemand(0.0);
        }

        conveyor.setTransportDemand(currentRobot.getConveyorConfig().getTransportIntakePercentOutput());
        conveyor.setFeedDemand(0);

        return this;
    }

    @Override
    public void finish() {

    }

}
