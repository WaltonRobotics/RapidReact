package frc.robot.robotState;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

import static frc.robot.RobotContainer.godSubsystem;

public class Intaking implements IState {
    private final Intake intake = godSubsystem.getIntake();
    private final Conveyor conveyor = godSubsystem.getConveyor();
    @Override
    public void initialize() {
        conveyor.setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
        intake.setIntakeControlState(Intake.IntakeControlState.VOLTAGE);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        //dummy voltage values for the following:
        if (intake.isLeftIntakeDeployed()) {
            intake.setLeftIntakeDemand(8.0);
        }

        if (intake.isRightIntakeDeployed()) {
            intake.setRightIntakeDemand(8.0);
        }

        conveyor.setFeedDemand(8.0);
        conveyor.setTransportDemand(8.0);

        return new Disabled();
    }

    @Override
    public void finish() {

    }

}
