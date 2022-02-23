package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class Intaking implements IState {

    private final Intake intake = godSubsystem.getIntake();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    @Override
    public void initialize() {
        intake.setIntakeControlState(Intake.IntakeControlState.VOLTAGE);
        conveyor.setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
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

        conveyor.setTransportDemand(currentRobot.getConveyorConfig().getTransportIntakeVoltage());
        conveyor.setFeedDemand(currentRobot.getConveyorConfig().getFeedIntakeVoltage());

        return this;
    }

    @Override
    public void finish() {

    }

}
