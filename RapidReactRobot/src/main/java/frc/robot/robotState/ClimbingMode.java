package frc.robot.robotState;

import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Superstructure;

import static frc.robot.RobotContainer.godSubsystem;

public class ClimbingMode implements IState {

    @Override
    public void initialize() {
        godSubsystem.setCurrentMode(Superstructure.CurrentMode.CLIMBING_MODE);
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
