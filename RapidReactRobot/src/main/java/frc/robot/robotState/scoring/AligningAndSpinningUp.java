package frc.robot.robotState.scoring;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.GenericHID;
import frc.robot.OI;
import frc.robot.commands.DriveCommand;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringMode;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.*;
import frc.robot.util.UtilMethods;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Shooter.kBarfVelocityRawUnits;
import static frc.robot.Constants.Shooter.kNudgeDownTimeSeconds;
import static frc.robot.Constants.VisionConstants.kAlignmentPipeline;
import static frc.robot.Constants.VisionConstants.kShootingAlignmentToleranceDegrees;
import static frc.robot.OI.*;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SPINNING_UP_SLOT;

public class AligningAndSpinningUp implements IState {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();
    private final PIDController controller = drivetrain.getConfig().getAutoAlignController();
    private final Shooter shooter = godSubsystem.getShooter();

    private double nudgeDownTimeout;

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

        godSubsystem.setCurrentTargetFlywheelVelocity(shooter.getEstimatedVelocityFromTarget());

        // Disable manual drive control
        DriveCommand.setIsEnabled(false);

        controller.reset();

        nudgeDownTimeout = godSubsystem.getCurrentTime() + kNudgeDownTimeSeconds;
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!OI.shootButton.get() && !OI.barfButton.get() && !overrideAutoAimAndShootButton.get()
                && !((godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToAlignAndShoot()))
                && !godSubsystem.needsToAutoReject()) {
            return new ScoringMode();
        }

        if (godSubsystem.needsToAutoReject()) {
            shooter.setFlywheelDemand(kBarfVelocityRawUnits);
        } else {
            shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());
        }

        double yaw = OI.yawScale.apply(godSubsystem.getRotateX());
        double omega = 0;

        // Ensure at least the minimum turn omega is supplied to the drivetrain to prevent stalling
        if (Math.abs(godSubsystem.getRotateX()) > yawScale.getDeadband()) {
            omega = Math.signum(yaw) * Math.max(Math.abs(yaw * drivetrain.getConfig().getMaxOmega()),
                    drivetrain.getConfig().getMinTurnOmega());
        }

        godSubsystem.handleAutoAlign(0, 0, omega, false);

        if (LimelightHelper.getTV() >= 1) {
            driveGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            driveGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        } else {
            driveGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0.15);
            driveGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0.15);
        }

//        if (godSubsystem.getCurrentTime() < nudgeDownTimeout) {
//            godSubsystem.handleIntaking();
//
//            godSubsystem.getConveyor().setTransportDemand(
//                    godSubsystem.getConveyor().getConfig().getTransportOuttakePercentOutput());
//            godSubsystem.getConveyor().setFeedDemand(
//                    godSubsystem.getConveyor().getConfig().getFeedOuttakePercentOutput());
//        } else {
//            godSubsystem.handleIntakingAndOuttaking();
//        }

        godSubsystem.handleIntakingAndOuttaking();

        if ((UtilMethods.isWithinTolerance(LimelightHelper.getTX(), 0, kShootingAlignmentToleranceDegrees)
                || overrideAutoAimAndShootButton.get())) {
            return new PreparingToShoot();
        }

        return this;
    }

    @Override
    public void finish() {
        drivetrain.move(0, 0, 0, false);

        driveGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
        driveGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
    }

}
