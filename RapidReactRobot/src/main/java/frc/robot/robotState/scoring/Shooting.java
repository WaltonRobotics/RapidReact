package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;

import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SHOOT_SLOT;
import static frc.robot.subsystems.Superstructure.targetFlyWheelVelocity;

public class Shooting implements IState {
    private final Shooter shooter = godSubsystem.getShooter();
    private final Intake intake = godSubsystem.getIntake();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    @Override
    public void initialize() {
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);
        shooter.setSelectedProfileSlot(SHOOT_SLOT);   //what is this?
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }
        //the following are dummy voltage values
        shooter.setFlywheelDemand(targetFlyWheelVelocity);

        //if closed loop error > threshold
        if(shooter.getFlywheelClosedLoopErrorNU() > 20){    //dummy threshold
            return new SpinningUp();
        }

        //if difference in time changed >= threshold
        if(shooter.getCurrentFPGATime() >= shooter.getLastAdjustableHoodChangeFPGATime()){
            //dummy conveyor voltages
            conveyor.setFeedDemand(10);
            conveyor.setTransportDemand(10);
        }

        if(!OI.shootButton.getAsBoolean()){
            return new ScoringMode();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
