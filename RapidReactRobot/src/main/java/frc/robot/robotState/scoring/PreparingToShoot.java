package frc.robot.robotState.scoring;

import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;

public class PreparingToShoot implements IState {

    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        godSubsystem.setCurrentTargetFlywheelVelocity(shooter.getEstimatedVelocityFromTarget());
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        return new SpinningUp();
    }

    @Override
    public void finish() {

    }

}
