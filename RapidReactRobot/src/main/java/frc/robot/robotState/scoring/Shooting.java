package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.commands.DriveCommand;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

import static frc.robot.Constants.Shooter.kRecoveryToleranceRawUnits;
import static frc.robot.OI.*;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SHOOTING_SLOT;

public class Shooting implements IState {

    private final Shooter shooter = godSubsystem.getShooter();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.OPEN_LOOP);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        shooter.setSelectedProfileSlot(SHOOTING_SLOT);
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        conveyor.setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);

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
            return new SpinningDown();
        }

        if (godSubsystem.isRobotMotionOverride()) {
            return new ShootWhileMoving();
        }

        shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

//        if (Math.abs(shooter.getFlywheelClosedLoopErrorNU()) > kShootingToleranceRawUnits) {
//            return new SpinningUp();
//        }

        godSubsystem.getDrivetrain().xLockSwerveDrive();

        if (intakeButton.get() || (godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToIntake())) {
            godSubsystem.handleIntaking();
        } else if (outtakeButton.get()) {
            godSubsystem.handleOuttaking();
        }

        double flywheelError =
                Math.abs(shooter.getFlywheelVelocityNU() - godSubsystem.getCurrentTargetFlywheelVelocity());

        // Wait for hood to move in position
        if ((shooter.getEstimatedHoodPosition() == shooter.getAdjustableHoodDutyCycleDemand()
                && flywheelError <= kRecoveryToleranceRawUnits) || overrideAutoAimAndShootButton.get()) {
            conveyor.setTransportDemand(conveyor.getConfig().getTransportShootPercentOutput());
            conveyor.setFeedDemand(conveyor.getConfig().getFeedShootPercentOutput());
        } else {
//            godSubsystem.handleTransportConveyorManualOverride();
            conveyor.setTransportDemand(conveyor.getConfig().getTransportIntakePercentOutput());
            godSubsystem.handleFeedConveyorManualOverride();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
