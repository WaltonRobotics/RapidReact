package frc.robot.robotState;

import frc.robot.Constants;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.PIDProfileSlots.kShooterDefaultIndex;
import static frc.robot.RobotContainer.godSubsystem;

import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.DEFAULT_SLOT;

public class Shooting implements IState {
    private final Shooter shooter = godSubsystem.getShooter();
    private final Intake intake = godSubsystem.getIntake();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    @Override
    public void initialize() {
        //TODO: set PID slot to shooting
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);
        shooter.setSelectedProfileSlot(DEFAULT_SLOT);   //what is this?
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }
        //the following are dummy voltage values
        shooter.setFlywheelDemand(10);
        if(shooter.getFlywheelClosedLoopErrorNU() > 20){    //dummy threshold
            return new SpinningUp();
        }
        if(shooter.getCurrentFPGATime() >= shooter.getLastAdjustableHoodChangeFPGATime()){
            conveyor.setFeedDemand(10);
            conveyor.setTransportDemand(10);
        }
        if(shooter.getFlywheelDemand() != 10) {
            return new SpinningUp();
        }
        return this;
    }

    @Override
    public void finish() {

    }

}
