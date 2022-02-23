package frc.robot.robotState.scoring;

import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.HoodPosition.SEVENTY_DEGREES;
import static frc.robot.subsystems.Superstructure.targetFlyWheelVelocity;

public class PreparingToShoot implements IState {
    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        //calculating targetVelocity
        if (shooter.getHoodPosition() == SEVENTY_DEGREES) {
            targetFlyWheelVelocity = shooter.getHoodTwoEstimatedVelocityFromTarget();
        } else {
            targetFlyWheelVelocity = shooter.getHoodOneEstimatedVelocityFromTarget();
        }
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
