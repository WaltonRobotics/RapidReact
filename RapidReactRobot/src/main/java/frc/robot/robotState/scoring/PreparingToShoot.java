package frc.robot.robotState.scoring;

import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.FieldConstants.kHoodCloseUpDistanceFeet;
import static frc.robot.RobotContainer.godSubsystem;

public class PreparingToShoot implements IState {

    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        // Re-adjust hood
        if (LimelightHelper.getDistanceToTargetFeet() <= kHoodCloseUpDistanceFeet) {
            shooter.setHoodPosition(Shooter.HoodPosition.SIXTY_DEGREES);
        } else {
            shooter.setHoodPosition(Shooter.HoodPosition.SEVENTY_DEGREES);
        }

        // Recalculate target velocity
        godSubsystem.setCurrentTargetFlywheelVelocity(shooter.getEstimatedVelocityFromTarget());
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        return new SpinningUp();
    }

    @Override
    public void finish() {

    }

}
