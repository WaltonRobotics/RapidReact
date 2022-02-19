package frc.robot.robotState;

import frc.robot.OI;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;

import static frc.robot.RobotContainer.godSubsystem;

public class Outtaking implements IState {
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
        if (intake.isLeftIntakeDeployStateDemand()) {
            intake.setLeftIntakeDemand(-8.0);
        }

        if (intake.isRightIntakeDeployStateDemand()) {
            intake.setRightIntakeDemand(-8.0);
        }
        conveyor.setFeedDemand(-8.0);
        conveyor.setTransportDemand(-8.0);

        if(!OI.outtakeButton.getAsBoolean()){
            return new Disabled();
        }
        return this;
    }

    @Override
    public void finish() {

    }

}
