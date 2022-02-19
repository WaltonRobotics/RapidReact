package frc.robot.robotState;

import frc.robot.OI;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.LimelightHelper;

import static frc.robot.RobotContainer.godSubsystem;

public class AdjustingHood implements IState {
    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        if (LimelightHelper.getDistanceToTargetMeters() <= 5) { //dummy value
            shooter.setRightAdjustableHoodDutyCycleDemand(10);
            shooter.setLeftAdjustableHoodDutyCycleDemand(10);
        } else {
            shooter.setRightAdjustableHoodDutyCycleDemand(20);
            shooter.setLeftAdjustableHoodDutyCycleDemand(20);
        }
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }
        //the following are dummy values
        if (!OI.shootButtonButton.getAsBoolean()) {
            return new Disabled();
        }
        return new Shooting();
    }

    @Override
    public void finish() {

    }

}
