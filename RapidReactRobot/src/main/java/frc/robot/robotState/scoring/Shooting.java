package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
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
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.DISABLED);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        shooter.setSelectedProfileSlot(SHOOTING_SLOT);
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        conveyor.setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.shootButton.get() && !OI.barfButton.get()) {
            return new SpinningDown();
        }

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        if (Math.abs(shooter.getFlywheelClosedLoopErrorNU()) > kShootingToleranceRawUnits) {
            return new SpinningUp();
        }

        // Wait for hood to move in position
        if (godSubsystem.getCurrentTime() >= shooter.getLastAdjustableHoodChangeFPGATime() + kHoodTransitionTimeSeconds) {
            conveyor.setTransportDemand(conveyor.getConfig().getTransportShootPercentOutput());
            conveyor.setFeedDemand(conveyor.getConfig().getFeedShootPercentOutput());
        } else {
            godSubsystem.handleTransportConveyorManualOverride();
            godSubsystem.handleFeedConveyorManualOverride();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
