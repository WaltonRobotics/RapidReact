package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Shooter.kHoodTransitionTimeSeconds;
import static frc.robot.Constants.Shooter.kShootingToleranceRawUnits;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SHOOTING_SLOT;

public class Shooting implements IState {

    private final Shooter shooter = godSubsystem.getShooter();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    @Override
    public void initialize() {
        shooter.setSelectedProfileSlot(SHOOTING_SLOT);
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        conveyor.setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.shootButton.get()) {
            return new ScoringMode();
        }

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        if (shooter.getFlywheelClosedLoopErrorNU() > kShootingToleranceRawUnits) {
            return new SpinningUp();
        }

        // Wait for hood to move in position
        if (godSubsystem.getCurrentTime() >= shooter.getLastAdjustableHoodChangeFPGATime() + kHoodTransitionTimeSeconds) {
            conveyor.setTransportDemand(conveyor.getConfig().getTransportShootVoltage());
            conveyor.setFeedDemand(conveyor.getConfig().getFeedShootVoltage());
        } else {
            conveyor.setTransportDemand(0);
            conveyor.setFeedDemand(0);
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
