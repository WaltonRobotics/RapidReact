package frc.robot.subsystems;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.auton.TurnToAngle;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.UtilMethods;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Climber.kPivotArmNudgeIncrementNU;
import static frc.robot.Constants.ContextFlags.kIsInShooterTuningMode;
import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DriverPreferences.*;
import static frc.robot.Constants.FieldConstants.*;
import static frc.robot.Constants.Shooter.kIdleVelocityRawUnits;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Constants.VisionConstants.kAlignmentToleranceDegrees;
import static frc.robot.OI.*;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.util.UtilMethods.monitorTemp;

public class Superstructure extends SubsystemBase {

    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();

    private boolean isEnabled = false;
    private CurrentMode currentMode = CurrentMode.SCORING_MODE;
    private ClimbingTargetRung selectedRung = ClimbingTargetRung.MID_RUNG;

    private double currentTargetFlywheelVelocity = 0;

    private boolean isInAuton = false;

    private boolean doesAutonNeedToTrackTarget = false;
    private boolean doesAutonNeedToIdleSpinUp = false;
    private boolean doesAutonNeedToIntake = false;
    private boolean doesAutonNeedToShoot = false;
    private boolean doesAutonNeedToAlignAndShoot = false;

    private final StateMachine stateMachine;

    public Superstructure() {
        stateMachine = new StateMachine("Superstructure", new Disabled());
    }

    public IState getCurrentState() {
        return stateMachine.getCurrentState();
    }

    public CurrentMode getCurrentMode() {
        return currentMode;
    }

    public void setCurrentMode(CurrentMode currentMode) {
        this.currentMode = currentMode;
    }

    public void toggleCurrentMode() {
        if (currentMode == CurrentMode.SCORING_MODE) {
            setCurrentMode(CurrentMode.CLIMBING_MODE);
        } else {
            setCurrentMode(CurrentMode.SCORING_MODE);
        }
    }

    public ClimbingTargetRung getSelectedRung() {
        return selectedRung;
    }

    public void setSelectedRung(ClimbingTargetRung rung) {
        this.selectedRung = rung;
    }

    public Drivetrain getDrivetrain() {
        return drivetrain;
    }

    public Intake getIntake() {
        return intake;
    }

    public Conveyor getConveyor() {
        return conveyor;
    }

    public Shooter getShooter() {
        return shooter;
    }

    public Climber getClimber() {
        return climber;
    }

    public boolean isEnabled() {
        return isEnabled;
    }

    public void setEnabled(boolean enabled) {
        isEnabled = enabled;
    }

    public double getCurrentTargetFlywheelVelocity() {
        return currentTargetFlywheelVelocity;
    }

    public void setCurrentTargetFlywheelVelocity(double currentTargetFlywheelVelocity) {
        this.currentTargetFlywheelVelocity = currentTargetFlywheelVelocity;
    }

    public boolean isInAuton() {
        return isInAuton;
    }

    public void setInAuton(boolean inAuton) {
        isInAuton = inAuton;
    }

    public boolean doesAutonNeedToIntake() {
        return doesAutonNeedToIntake;
    }

    public void setDoesAutonNeedToIntake(boolean needsToIntake) {
        this.doesAutonNeedToIntake = needsToIntake;
    }

    public boolean doesAutonNeedToShoot() {
        return doesAutonNeedToShoot;
    }

    public void setDoesAutonNeedToShoot(boolean needsToShoot) {
        this.doesAutonNeedToShoot = needsToShoot;
    }

    public boolean doesAutonNeedToAlignAndShoot() {
        return doesAutonNeedToAlignAndShoot;
    }

    public void setDoesAutonNeedToAlignAndShoot(boolean doesAutonNeedToAlignAndShoot) {
        this.doesAutonNeedToAlignAndShoot = doesAutonNeedToAlignAndShoot;
    }

    public double getCurrentTime() {
        return Timer.getFPGATimestamp();
    }

    public boolean isPivotManualOverride() {
        return dangerButton.get() && Math.abs(manipulationGamepad.getLeftX()) > kPivotManualOverrideDeadband;
    }

    public boolean isExtensionManualOverride() {
        return dangerButton.get() && Math.abs(manipulationGamepad.getRightY()) > kExtensionManualOverrideDeadband;
    }

    public boolean isClimbingMovementOverride() {
        return Math.abs(driveGamepad.getLeftY()) > forwardScale.getDeadband()
                || Math.abs(driveGamepad.getLeftX()) > strafeScale.getDeadband()
                || Math.abs(driveGamepad.getRightX()) > yawScale.getDeadband();
    }

    public void handleTransportConveyorManualOverride() {
        if (OI.overrideTransportConveyorButton.get()) {
            getConveyor().setTransportDemand(conveyor.getConfig().getTransportIntakePercentOutput());
        } else {
            getConveyor().setTransportDemand(0);
        }
    }

    public void handleFeedConveyorManualOverride() {
        if (OI.overrideFeedConveyorButton.get()) {
            getConveyor().setFeedDemand(conveyor.getConfig().getFeedShootPercentOutput());
        } else {
            getConveyor().setFeedDemand(0);
        }
    }

    public void handleIntaking() {
        if (intake.isLeftIntakeDeployed()) {
            double configOutput = intake.getConfig().getLeftIntakePercentOutput();

            if (kIsInTuningMode) {
                intake.setLeftIntakeDemand(SmartDashboard.getNumber(kLeftIntakePercentOutputKey, configOutput));
            } else {
                intake.setLeftIntakeDemand(configOutput);
            }
        } else {
            intake.setLeftIntakeDemand(0);
        }

        if (intake.isRightIntakeDeployed()) {
            double configOutput = intake.getConfig().getRightIntakePercentOutput();

            if (kIsInTuningMode) {
                intake.setRightIntakeDemand(SmartDashboard.getNumber(kRightIntakePercentOutputKey, configOutput));
            } else {
                intake.setRightIntakeDemand(configOutput);
            }
        } else {
            intake.setRightIntakeDemand(0);
        }
    }

    public void handleIntakingWithConveyor() {
        handleIntaking();

        conveyor.setTransportDemand(conveyor.getConfig().getTransportIntakePercentOutput());
        conveyor.setFeedDemand(0);
    }

    public void handleOuttaking() {
        if (intake.isLeftIntakeDeployed()) {
            intake.setLeftIntakeDemand(intake.getConfig().getLeftOuttakePercentOutput());
        } else {
            intake.setLeftIntakeDemand(0);
        }

        if (intake.isRightIntakeDeployed()) {
            intake.setRightIntakeDemand(intake.getConfig().getRightOuttakePercentOutput());
        } else {
            intake.setRightIntakeDemand(0);
        }
    }

    public void handleOuttakingWithConveyor() {
        handleOuttaking();

        conveyor.setTransportDemand(currentRobot.getConveyorConfig().getTransportOuttakePercentOutput());
        conveyor.setFeedDemand(currentRobot.getConveyorConfig().getFeedOuttakePercentOutput());
    }

    public void handleIntakingAndOuttaking() {
        if (OI.intakeButton.get()
                || (isInAuton() && doesAutonNeedToIntake())) {
            handleIntakingWithConveyor();
        } else if (OI.outtakeButton.get()) {
            handleOuttakingWithConveyor();
        } else {
            intake.setLeftIntakeDemand(0);
            intake.setRightIntakeDemand(0);

            handleTransportConveyorManualOverride();
            handleFeedConveyorManualOverride();
        }
    }

    public void handlePivotManualOverride() {
        if (isPivotManualOverride()) {
            climber.setPivotControlState(Climber.ClimberControlState.OPEN_LOOP);

            double pivotJoystick = manipulationGamepad.getLeftX()
                    * climber.getConfig().getManualPivotPercentOutputLimit();

            climber.setPivotPercentOutputDemand(pivotJoystick);
        } else {
            climber.setPivotControlState(Climber.ClimberControlState.AUTO);

            if (nudgePivotArmCCWButton.isRisingEdge()) {
                double currentPivotAngle = climber.getPivotPositionDemandNU();

                climber.setPivotPositionDemandNU(currentPivotAngle + kPivotArmNudgeIncrementNU);
            }

            if (nudgePivotArmCWButton.isRisingEdge()) {
                double currentPivotAngle = climber.getPivotPositionDemandNU();

                climber.setPivotPositionDemandNU(currentPivotAngle - kPivotArmNudgeIncrementNU);
            }
        }
    }

    public void handleExtensionManualOverride() {
        if (isExtensionManualOverride()) {
            climber.setExtensionControlState(Climber.ClimberControlState.OPEN_LOOP);

            double extensionJoystick = manipulationGamepad.getRightY();

            extensionJoystick = UtilMethods.limitMagnitude(extensionJoystick,
                    climber.getConfig().getExtensionManualPercentOutputLimit());

            climber.setExtensionPercentOutputDemand(extensionJoystick);
        } else {
            climber.setExtensionControlState(Climber.ClimberControlState.AUTO);
        }
    }

    public void handleIdleSpinUp() {
        if (idleSpinUpButton.get() || (isInAuton() && doesAutonNeedToIdleSpinUp())) {
            shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);
            shooter.setFlywheelDemand(kIdleVelocityRawUnits);
        } else {
            shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);
            shooter.setFlywheelDemand(0);
        }
    }

    public void handleTrackTarget() {
        if (LimelightHelper.getTV() >= 1) {
//            double distanceFromTarget = LimelightHelper.getDistanceToTargetFeet();
//
//            if (distanceFromTarget < kSpinUpFlywheelDistanceFromHub) {
//                shooter.setShooterControlState(Shooter.ShooterControlState.VELOCITY);
//                shooter.setFlywheelDemand(kIdleVelocityRawUnits);
//            }

            if (!kIsInShooterTuningMode) {
                double hoodAngle = shooter.getEstimatedHoodAngleFromTarget();
                shooter.setAdjustableHoodDutyCycleDemand(hoodAngle);
            }
        }

        handleIdleSpinUp();
    }

    public boolean doesAutonNeedToIdleSpinUp() {
        return doesAutonNeedToIdleSpinUp;
    }

    public void setDoesAutonNeedToIdleSpinUp(boolean doesAutonNeedToIdleSpinUp) {
        this.doesAutonNeedToIdleSpinUp = doesAutonNeedToIdleSpinUp;
    }

    public boolean doesAutonNeedToTrackTarget() {
        return doesAutonNeedToTrackTarget;
    }

    public void setDoesAutonNeedToTrackTarget(boolean doesAutonNeedToTrackTarget) {
        this.doesAutonNeedToTrackTarget = doesAutonNeedToTrackTarget;
    }

    public void updateShuffleboard() {
        drivetrain.updateShuffleboard();
        intake.updateShuffleboard();
        conveyor.updateShuffleboard();
        shooter.updateShuffleboard();
        climber.updateShuffleboard();

        SmartDashboard.putString(kDriverSelectedRungKey, getSelectedRung().name());
        SmartDashboard.putNumber(kShooterCurrentTargetVelocityKey, getCurrentTargetFlywheelVelocity());

        boolean hasTarget = LimelightHelper.getTV() >= 1;
        double limelightDistance = LimelightHelper.getDistanceToTargetFeet();

        SmartDashboard.putNumber(kLimelightDistanceFeetKey, limelightDistance);

        SmartDashboard.putBoolean(kDriverIsAlignedKey,
                hasTarget &&
                        UtilMethods.isWithinTolerance(LimelightHelper.getTX(), 0, kAlignmentToleranceDegrees));

        SmartDashboard.putBoolean(kDriverIsMoneyShotKey,
                hasTarget && UtilMethods.isWithinTolerance(limelightDistance, kMoneyShotDistance,
                        kMoneyShotTolerance));
    }

    public void monitorTemperatures() {
        monitorTemp(climber.getPivotTemp(),60, "Pivot Overheating");
        monitorTemp(climber.getExtensionTemp(), 70, "Extension Overheating" );
        monitorTemp(shooter.getMasterTemp(), 70, "Right Shooter Overheating" );
        monitorTemp(shooter.getSlaveTemp(), 70, "Left Shooter Overheating");
        monitorTemp(drivetrain.getLeftFrontDriveTemp(),70, "Left Front Talon Overheating");
        monitorTemp(drivetrain.getLeftBackDriveTemp(),70, "Left Back Talon Overheating");
        monitorTemp(drivetrain.getRightFrontDriveTemp(),70, "Right Front Talon Overheating");
        monitorTemp(drivetrain.getRightBackDriveTemp(),70, "Right Back Talon Overheating");
        monitorTemp(drivetrain.getLeftFrontTurnTemp(),60, "Left Front NEO Overheating");
        monitorTemp(drivetrain.getLeftBackTurnTemp(),60, "Left Back NEO Overheating");
        monitorTemp(drivetrain.getRightFrontTurnTemp(),60, "Right Front NEO Overheating");
        monitorTemp(drivetrain.getRightBackTurnTemp(),60, "Right Back NEO Overheating");
    }

    @Override
    public void periodic() {
        stateMachine.run();
    }

    public enum CurrentMode {
        SCORING_MODE, CLIMBING_MODE
    }

    public enum ClimbingTargetRung {
        MID_RUNG, HIGH_RUNG
    }

}
