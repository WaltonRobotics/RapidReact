package frc.robot.robotState;

import frc.robot.stateMachine.IState;

import static frc.robot.RobotContainer.godSubsystem;

public class Intaking implements IState {
    @Override
    public void initialize() {

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
