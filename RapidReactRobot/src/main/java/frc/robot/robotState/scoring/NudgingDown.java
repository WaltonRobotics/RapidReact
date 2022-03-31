package frc.robot.robotState.scoring;

import frc.robot.OI;
import frc.robot.commands.DriveCommand;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.robotState.ScoringModeTransition;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Shooter.kNudgeDownTimeSeconds;
import static frc.robot.Constants.VisionConstants.kAlignmentPipeline;
import static frc.robot.OI.barfButton;
import static frc.robot.OI.overrideAutoAimAndShootButton;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SPINNING_UP_SLOT;

public class NudgingDown implements IState {

    private final Shooter shooter = godSubsystem.getShooter();
    private double timeout;

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.OPEN_LOOP);
        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        LimelightHelper.setLEDMode(true);
        LimelightHelper.setPipeline(kAlignmentPipeline);

        shooter.setSelectedProfileSlot(SPINNING_UP_SLOT);
        shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        // Disable manual drive control
        DriveCommand.setIsEnabled(false);

        timeout = godSubsystem.getCurrentTime() + kNudgeDownTimeSeconds;
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.shootButton.get() && !barfButton.get() && !overrideAutoAimAndShootButton.get()
                && !((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToShoot()))
                && !((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToAlignAndShoot()))) {
            return new ScoringModeTransition();
        }

        godSubsystem.handleIdleSpinUp();

        godSubsystem.handleIntaking();

        godSubsystem.getConveyor().setTransportDemand(
                godSubsystem.getConveyor().getConfig().getTransportOuttakePercentOutput());
        godSubsystem.getConveyor().setFeedDemand(
                godSubsystem.getConveyor().getConfig().getFeedOuttakePercentOutput());

        if (!godSubsystem.isInAuton()) {
            godSubsystem.getDrivetrain().xLockSwerveDrive();
        }

        if (godSubsystem.getCurrentTime() >= timeout) {
            return new PreparingToShoot();
        }

        return this;
    }

    @Override
    public void finish() {

    }

}
