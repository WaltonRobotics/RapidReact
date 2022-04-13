package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.commands.DriveCommand;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringModeTransition;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Shooter.kSpinningUpToleranceRawUnits;
import static frc.robot.OI.overrideAutoAimAndShootButton;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SPINNING_UP_SLOT;

public class SpinningUp implements IState {

    private final Shooter shooter = godSubsystem.getShooter();

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.OPEN_LOOP);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        shooter.setSelectedProfileSlot(SPINNING_UP_SLOT);
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        DriveCommand.setIsEnabled(false);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.shootButton.get() && !OI.barfButton.get() && !overrideAutoAimAndShootButton.get()
                && !((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToShoot()))
                && !((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToAlignAndShoot()))
                && !((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToBarf()))) {
            return new ScoringModeTransition();
        }

        if (godSubsystem.isRobotMotionOverride()) {
            return new ShootWhileMoving();
        }

        godSubsystem.getDrivetrain().xLockSwerveDrive();

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        godSubsystem.handleIntakingAndOuttaking();

        if ((Math.abs(godSubsystem.getCurrentTargetFlywheelVelocity() - shooter.getFlywheelVelocityNU())
                <= kSpinningUpToleranceRawUnits) || overrideAutoAimAndShootButton.get()) {
            return new Shooting();
        }

        godSubsystem.getConveyor().setTransportDemand(godSubsystem.getConveyor().getConfig().getTransportIntakePercentOutput());
        godSubsystem.getConveyor().setFeedDemand(0);

        return this;
    }

    @Override
    public void finish() {

    }

}
