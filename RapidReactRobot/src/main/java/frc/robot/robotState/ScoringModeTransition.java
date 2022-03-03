package frc.robot.robotState;

import frc.robot.stateMachine.IState;

import static frc.robot.RobotContainer.godSubsystem;

public class ScoringModeTransition implements IState {

    @Override
    public void initialize() {
        // Engage climber locks
        // Engage climber disc brake
        godSubsystem.getClimber().setClimberLockStateDemand(false);

        godSubsystem.getClimber().setClimberDiscBrakeStateDemand(false);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        return new ScoringMode();
    }

    @Override
    public void finish() {

    }

}
