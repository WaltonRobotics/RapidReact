package frc.robot.robotState.scoring;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.commands.DriveCommand;
import frc.robot.robotState.Disabled;
import frc.robot.robotState.ScoringModeTransition;
import frc.robot.stateMachine.IState;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Conveyor;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.util.UtilMethods;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.DriverPreferences.kMotionCorrectShooting;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.OI.*;
import static frc.robot.OI.outtakeButton;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Shooter.ShooterProfileSlot.SHOOTING_SLOT;

public class ShootWhileMoving implements IState {

    private final Shooter shooter = godSubsystem.getShooter();
    private final Conveyor conveyor = godSubsystem.getConveyor();

    @Override
    public void initialize() {
        godSubsystem.getIntake().setIntakeControlState(Intake.IntakeControlState.OPEN_LOOP);
        godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.DISABLED);
        godSubsystem.getClimber().setExtensionControlState(Climber.ClimberControlState.DISABLED);

        godSubsystem.getShooter().setSelectedProfileSlot(SHOOTING_SLOT);
        godSubsystem.getShooter().setShooterControlState(Shooter.ShooterControlState.VELOCITY);

        godSubsystem.getConveyor().setConveyorControlState(Conveyor.ConveyorControlState.OPEN_LOOP);
        godSubsystem.getConveyor().setTransportDemand(0.0);
        godSubsystem.getConveyor().setFeedDemand(0.0);

        DriveCommand.setIsEnabled(false);
    }

    @Override
    public IState execute() {
        if (!godSubsystem.isEnabled()) {
            return new Disabled();
        }

        if (!godSubsystem.isRobotMotionOverride() || !shootButton.get()) {
            return new ScoringModeTransition();
        }

        godSubsystem.handleTrackTarget();

        double vx = godSubsystem.getVx();
        double vy = godSubsystem.getVy();

        double shooterVelocityNU = godSubsystem.getShooter().getEstimatedVelocityFromTarget();
        double shooterMPS = shooterVelocityNU * kFlywheelNUToMPS
                * godSubsystem.getShooter().getHoodAngleFromHorizontal().getCos();

        Rotation2d robotTarget = godSubsystem.getDrivetrain().getHeading().minus(
                Rotation2d.fromDegrees(LimelightHelper.getTX()));

        if (LimelightHelper.getTV() >= 1) {
            ChassisSpeeds robotSpeeds = godSubsystem.getDrivetrain().getRobotRelativeSpeeds();

            if (Math.abs(robotSpeeds.vxMetersPerSecond) > 0.1
                    || Math.abs(robotSpeeds.vyMetersPerSecond) > 0.1) {
                double correctionXSpeed = shooterMPS * robotTarget.getCos() - robotSpeeds.vxMetersPerSecond;
                double correctionYSpeed = shooterMPS * robotTarget.getSin() - robotSpeeds.vyMetersPerSecond;

                robotTarget = new Rotation2d(Math.atan2(correctionYSpeed, correctionXSpeed));

                shooterMPS = Math.sqrt(correctionXSpeed * correctionXSpeed + correctionYSpeed * correctionYSpeed);
                shooterVelocityNU = shooterMPS / kFlywheelNUToMPS;
            }
        }

        // Set shooter flywheel velocity

        godSubsystem.setCurrentTargetFlywheelVelocity(shooterVelocityNU);

        godSubsystem.getShooter().setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());

        // Set robot vx, vy and make it face robotTarget

        double thetaError = godSubsystem.getDrivetrain().faceDirection(vx, vy, robotTarget, false);

        double flywheelError =
                Math.abs(shooter.getFlywheelVelocityNU() - godSubsystem.getCurrentTargetFlywheelVelocity());

        if ((UtilMethods.isWithinTolerance(thetaError, 0, kShootingToleranceRawUnits)
                && shooter.getEstimatedHoodPosition() == shooter.getAdjustableHoodDutyCycleDemand()
                && flywheelError <= kSpinningUpToleranceRawUnits) || overrideAutoAimAndShootButton.get()) {
            conveyor.setTransportDemand(conveyor.getConfig().getTransportShootPercentOutput());
            conveyor.setFeedDemand(conveyor.getConfig().getFeedShootPercentOutput());
        } else {
            godSubsystem.handleTransportConveyorManualOverride();
            godSubsystem.handleFeedConveyorManualOverride();
        }

        if (intakeButton.get() || (godSubsystem.isInAuton() && godSubsystem.doesAutonNeedToIntake())) {
            godSubsystem.handleIntaking();
        } else if (outtakeButton.get()) {
            godSubsystem.handleOuttaking();
        }

        return this;
    }

    @Override
    public void finish() {
        DriveCommand.setIsEnabled(true);
    }

}
