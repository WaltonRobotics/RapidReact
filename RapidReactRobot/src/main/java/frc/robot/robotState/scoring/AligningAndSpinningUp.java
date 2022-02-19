package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.commands.AimCommandLime;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.HoodState.SEVENTY_DEGREES;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SPIN_UP_SLOT;
import static frc.robot.subsystems.Superstructure.targetFlyWheelVelocity;

public class AligningAndSpinningUp implements IState {
    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        shooter.setSelectedProfileSlot(SPIN_UP_SLOT);
        //calculating targetVelocity
        if(shooter.getHoodState() == SEVENTY_DEGREES){
            targetFlyWheelVelocity = shooter.getHoodTwoEstimatedVelocityFromTarget();
        }
        else{
            targetFlyWheelVelocity = shooter.getHoodOneEstimatedVelocityFromTarget();
        }
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        shooter.setFlywheelDemand(targetFlyWheelVelocity);

        new AimCommandLime();   //isFinished() will handle tolerance

        if(!OI.shootButton.getAsBoolean()){
            return new ScoringMode();
        }

        return new PreparingToShoot();
    }

    @Override
    public void finish() {

    }

}
