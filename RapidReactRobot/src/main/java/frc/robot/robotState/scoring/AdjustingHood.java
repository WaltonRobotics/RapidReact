package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.LimelightHelper;

import static frc.robot.RobotContainer.godSubsystem;

public class AdjustingHood implements IState {
    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        if (LimelightHelper.getDistanceToTargetFeet() <= 15) { //dummy value
            shooter.setRightAdjustableHoodDutyCycleDemand(10);
            shooter.setLeftAdjustableHoodDutyCycleDemand(10);
            shooter.setHoodState(Shooter.HoodState.SIXTY_DEGREES);
        } else {
            shooter.setRightAdjustableHoodDutyCycleDemand(20);
            shooter.setLeftAdjustableHoodDutyCycleDemand(20);
            shooter.setHoodState(Shooter.HoodState.SEVENTY_DEGREES);
        }
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }
        //the following are dummy values
        if (!OI.shootButton.getAsBoolean()) {
            return new ScoringMode();
        }
        return new Shooting();
    }

    @Override
    public void finish() {

    }

}
