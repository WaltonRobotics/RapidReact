package frc.robot.robotState;

import frc.robot.commands.AimCommandLime;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.LimelightHelper;

import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.HoodState.SEVENTY_DEGREES;
import static frc.robot.subsystems.Shooter.HoodState.SIXTY_DEGREES;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SPIN_UP_SLOT;

public class AligningAndSpinningUp implements IState {
    private final Shooter shooter = godSubsystem.getShooter();
    private double targetVelocity;

    @Override
    public void initialize() {
        targetVelocity = LimelightHelper.getDistanceToTargetFeet() * 1.5;   //dummy calculation
        shooter.setSelectedProfileSlot(SPIN_UP_SLOT);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if(shooter.getHoodState() == SEVENTY_DEGREES){
            shooter.setFlywheelDemand(shooter.getHoodTwoEstimatedVelocityFromTarget());
        }
        else{
            shooter.setFlywheelDemand(shooter.getHoodOneEstimatedVelocityFromTarget());
        }
        new AimCommandLime();   //isFinished() will handle tolerance


        return new PreparingToShoot();
    }

    @Override
    public void finish() {

    }

}
