package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SPINNING_UP_SLOT;
import static frc.robot.subsystems.Superstructure.targetFlyWheelVelocity;

public class SpinningUp implements IState {
    private final Shooter shooter = godSubsystem.getShooter();
    private final Intake intake = godSubsystem.getIntake();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    @Override
    public void initialize() {
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);
        shooter.setSelectedProfileSlot(SPINNING_UP_SLOT);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }
        shooter.setFlywheelDemand(targetFlyWheelVelocity);

        if (shooter.getFlywheelClosedLoopErrorNU() <= 10) {  //dummy threshold
            return new Shooting();
        }
        if (!OI.shootButton.getAsBoolean()) {
            return new ScoringMode();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
