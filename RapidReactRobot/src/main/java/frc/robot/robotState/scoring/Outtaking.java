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

public class Outtaking implements IState {

    private final Intake intake = godSubsystem.getIntake();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    @Override
    public void initialize() {
        intake.setIntakeControlState(Intake.IntakeControlState.VOLTAGE);
        conveyor.setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.DISABLED);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.outtakeButton.get()) {
            return new ScoringMode();
        }

        if (intake.isLeftIntakeDeployed()) {
            intake.setLeftIntakeDemand(currentRobot.getIntakeConfig().getLeftOuttakeVoltage());
        } else {
            intake.setLeftIntakeDemand(0);
        }

        if (intake.isRightIntakeDeployed()) {
            intake.setRightIntakeDemand(currentRobot.getIntakeConfig().getRightOuttakeVoltage());
        } else {
            intake.setRightIntakeDemand(0);
        }

        conveyor.setTransportDemand(currentRobot.getConveyorConfig().getTransportOuttakeVoltage());
        conveyor.setFeedDemand(currentRobot.getConveyorConfig().getFeedOuttakeVoltage());

        return this;
    }

    @Override
    public void finish() {

    }

}
