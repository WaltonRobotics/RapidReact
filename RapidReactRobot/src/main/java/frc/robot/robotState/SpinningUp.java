package frc.robot.robotState;

import frc.robot.OI;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.DEFAULT_SLOT;

public class SpinningUp implements IState {
    private final Shooter shooter = godSubsystem.getShooter();
    private final Intake intake = godSubsystem.getIntake();
    private final Conveyor conveyor = godSubsystem.getConveyor();
    @Override
    public void initialize() {
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);
        shooter.setSelectedProfileSlot(DEFAULT_SLOT);   //what is this?
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }
        shooter.setFlywheelDemand(10);  //dummy voltage
        if(shooter.getFlywheelClosedLoopErrorNU() <= 10) {  //dummy threshold
            return new Shooting();
        }
        if(!OI.shootButtonButton.getAsBoolean()){
            return new Disabled();
        }
        return this;
    }

    @Override
    public void finish() {

    }

}
