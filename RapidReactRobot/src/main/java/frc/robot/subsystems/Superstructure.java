package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.OI;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.auton.LiveDashboardHelper;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.UtilMethods;
import frc.robot.vision.LimelightHelper;

import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kLeftRumble;
import static edu.wpi.first.wpilibj.GenericHID.RumbleType.kRightRumble;
import static frc.robot.Constants.Climber.kPivotArmNudgeIncrementNU;
import static frc.robot.Constants.ContextFlags.*;
import static frc.robot.Constants.DriverPreferences.*;
import static frc.robot.Constants.FieldConstants.kCenterOfHubPose;
import static frc.robot.Constants.Shooter.kFlywheelNUToMPS;
import static frc.robot.Constants.Shooter.kIdleVelocityRawUnits;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Constants.VisionConstants.*;
import static frc.robot.OI.*;
import static frc.robot.RobotContainer.*;
import static frc.robot.util.UtilMethods.monitorTemp;

public class Superstructure extends SubsystemBase {

    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();
//    private final IndicatorLights lights = new IndicatorLights();

    private boolean isEnabled = false;
    private CurrentMode currentMode = CurrentMode.SCORING_MODE;
    private ClimbingTargetRung selectedRung = ClimbingTargetRung.HIGH_RUNG;

    private double currentTargetFlywheelVelocity = 0;

    private boolean isInAuton = false;
    private boolean isInPitCheckMode = false;
    private boolean isAutoAligning = false;

    private boolean doesAutonNeedToTrackTarget = false;
    private boolean doesAutonNeedToIdleSpinUp = false;
    private boolean doesAutonNeedToIntake = false;
    private boolean doesAutonNeedToShoot = false;
    private boolean doesAutonNeedToAlignAndShoot = false;
    private boolean doesAutonNeedToBarf = false;

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

//    public IndicatorLights getLights(){
//        return lights;
//    }

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

    public boolean isInPitCheckMode() {
        return isInPitCheckMode;
    }

    public void setIsInPitCheckMode(boolean inPitCheckMode) {
        isInPitCheckMode = inPitCheckMode;
    }

    public boolean isAutoAligning() {
        return isAutoAligning;
    }

    public void setAutoAligning(boolean autoAligning) {
        isAutoAligning = autoAligning;
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

    public boolean doesAutonNeedToBarf() {
        return doesAutonNeedToBarf;
    }

    public void setDoesAutonNeedToBarf(boolean doesAutonNeedToBarf) {
        this.doesAutonNeedToBarf = doesAutonNeedToBarf;
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

    /**
     * purple if both intakes, red if right intake
     * blue if left intake, green if aligned
     * no color else wise
     */
//    public void handleLEDLights() {
//        if(intake.isLeftIntakeDeployed() && intake.isRightIntakeDeployed()){
//            lights.setPurple();
//        }
//        else if(intake.isRightIntakeDeployed()){
//            lights.setRed();
//        }
//        else if(intake.isLeftIntakeDeployed()){
//            lights.setBlue();
//        }
//        else{
//            if(LimelightHelper.getTV() >= 1 &&
//                    UtilMethods.isWithinTolerance(LimelightHelper.getTX(), 0, kAlignmentToleranceDegrees)){
//                lights.setGreen();
//            }
//            else{
//                lights.setOff();
//            }
//        }
//    }

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

        if (kIsInShooterTuningMode) {
            shooter.setAdjustableHoodDutyCycleDemand(
                    SmartDashboard.getNumber(kShooterHoodPositionSetpointKey, 0.0));
        }

        handleIdleSpinUp();
    }

    // Takes care of reflecting the robot position and returns the actual pose of the robot on the field
    // This is done because red alliance trajectories and blue alliance trajectories use the same points
    public Pose2d getAllianceSpecificPose() {
        if (getInferredAllianceColor() == AllianceColor.BLUE) {
            return drivetrain.getPoseMeters();
        } else {
            return LiveDashboardHelper.reflectPose(drivetrain.getPoseMeters());
        }
    }

    public Rotation2d getEstimatedAngleToHub() {
        Pose2d targetRobotRelative = kCenterOfHubPose.relativeTo(getAllianceSpecificPose());

        return new Rotation2d(-Math.atan2(targetRobotRelative.getY(), targetRobotRelative.getX()));
    }

    public double handleAutoAlign(double vx, double vy, double manualOmega, boolean isFieldRelative,
                                  boolean useOdometryBackup) {
        double turnRate;
        double headingError = 0;

        if (LimelightHelper.getTV() >= 1) {
            headingError = LimelightHelper.getTX();
            turnRate = drivetrain.getConfig().getAutoAlignController().calculate(headingError, 0.0);

            turnRate = Math.signum(turnRate) * UtilMethods.limitRange(
                    Math.abs(turnRate), drivetrain.getConfig().getMinTurnOmega(),
                    drivetrain.getConfig().getMaxFaceDirectionOmega());

            if (Math.abs(headingError) < kAutoAlignToleranceDegrees) {
                turnRate = 0;
            }
        } else if (useOdometryBackup && Math.abs(getOmega()) <= yawScale.getDeadband()) {
            headingError = UtilMethods.restrictAngle(getEstimatedAngleToHub().getDegrees(), -180, 180);

            turnRate = drivetrain.getConfig().getAutoAlignController().calculate(headingError, 0.0);

            turnRate = Math.signum(turnRate) * UtilMethods.limitRange(
                    Math.abs(turnRate), drivetrain.getConfig().getMinTurnOmega(),
                    drivetrain.getConfig().getMaxFaceDirectionOmega());

            if (Math.abs(headingError) < kAutoAlignToleranceDegrees) {
                turnRate = 0;
            }
        } else {
            turnRate = manualOmega;
        }

        SmartDashboard.putNumber(kLimelightAlignOmegaOutputKey, turnRate);

        SmartDashboard.putNumber(kLimelightAlignErrorDegrees,
                drivetrain.getConfig().getAutoAlignController().getPositionError());

        drivetrain.move(vx, vy, turnRate, isFieldRelative);

        return headingError;
    }

    public void handleMotionCorrect() {
        ChassisSpeeds robotSpeeds = godSubsystem.getDrivetrain().getRobotRelativeSpeeds();

        if (LimelightHelper.getTV() >= 1 && (Math.abs(robotSpeeds.vxMetersPerSecond) > 0.1
                || Math.abs(robotSpeeds.vyMetersPerSecond) > 0.1) && kMotionCorrectShooting) {
            double hoodAngleCos = godSubsystem.getShooter().getHoodAngleFromHorizontal().getCos();

            double shooterVelocityNU = godSubsystem.getShooter().getEstimatedVelocityFromTarget();
            double shooterMPS = shooterVelocityNU * kFlywheelNUToMPS
                    * hoodAngleCos;

            Rotation2d robotTarget = Rotation2d.fromDegrees(-LimelightHelper.getTX());

            double correctionXSpeed = shooterMPS * robotTarget.getCos() - robotSpeeds.vxMetersPerSecond;
            double correctionYSpeed = shooterMPS * robotTarget.getSin() - robotSpeeds.vyMetersPerSecond;

            robotTarget = new Rotation2d(Math.atan2(correctionYSpeed, correctionXSpeed));

            shooterMPS = Math.sqrt(correctionXSpeed * correctionXSpeed + correctionYSpeed * correctionYSpeed);
            shooterVelocityNU = shooterMPS / kFlywheelNUToMPS * (1.0 / hoodAngleCos);

            shooter.setFlywheelDemand(shooterVelocityNU);
            godSubsystem.getDrivetrain().faceDirection(0, 0,
                    godSubsystem.getDrivetrain().getHeading().plus(robotTarget), false, false);
        } else {
            shooter.setFlywheelDemand(godSubsystem.getCurrentTargetFlywheelVelocity());
            godSubsystem.getDrivetrain().xLockSwerveDrive();
        }
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

    public double getVx() {
        double forward = OI.forwardScale.apply(getForward());
        return forward * drivetrain.getConfig().getMaxSpeedMetersPerSecond();
    }

    public double getVy() {
        double strafe = OI.strafeScale.apply(getStrafe());
        return strafe * drivetrain.getConfig().getMaxSpeedMetersPerSecond();
    }

    public double getOmega() {
        double yaw = OI.yawScale.apply(getRotateX());

        double omega = 0;

        // Ensure at least the minimum turn omega is supplied to the drivetrain to prevent stalling
        if (Math.abs(getRotateX()) > yawScale.getDeadband()) {
            omega = Math.signum(yaw) * Math.max(Math.abs(yaw * drivetrain.getConfig().getMaxOmega()),
                    drivetrain.getConfig().getMinTurnOmega());
        }

        return omega;
    }

    public boolean isRobotMotionOverride() {
        return (Math.abs(getVx()) > 0.3
                || Math.abs(getVy()) > 0.3) && kMotionCorrectShooting;
    }

    public void handleClimbTime() {
//        double matchTime = DriverStation.getMatchTime();
//
//        if(DriverStation.isTeleopEnabled() && matchTime <= 25) {
//            manipulationGamepad.setRumble(kLeftRumble, 0.15);
//            manipulationGamepad.setRumble(kRightRumble, 0.15);
//        } else if (DriverStation.isTeleopEnabled() && matchTime <= 23 && matchTime >= 22) {
//            manipulationGamepad.setRumble(kLeftRumble, 0);
//            manipulationGamepad.setRumble(kRightRumble, 0);
//        }
    }

    public void updateShuffleboard() {
        if (!kIsInCompetition) {
            conveyor.updateShuffleboard();
        }

        intake.updateShuffleboard();

        if (!isInAuton) {
            drivetrain.updateShuffleboard();
        }

        shooter.updateShuffleboard();
        climber.updateShuffleboard();

        SmartDashboard.putString(kDriverSelectedRungKey, getSelectedRung().name());
        SmartDashboard.putNumber(kShooterCurrentTargetVelocityKey, getCurrentTargetFlywheelVelocity());

        boolean hasTarget = LimelightHelper.getTV() >= 1;
        double limelightDistance = LimelightHelper.getDistanceToTargetFeet();

        SmartDashboard.putNumber(kLimelightDistanceFeetKey, limelightDistance);

        SmartDashboard.putBoolean(kDriverIsAlignedKey,
                hasTarget &&
                        UtilMethods.isWithinTolerance(LimelightHelper.getTX(), 0, kShootingAlignmentToleranceDegrees));

//        SmartDashboard.putBoolean(kDriverIsMoneyShotKey,
//                hasTarget && UtilMethods.isWithinTolerance(limelightDistance, kMoneyShotDistance,
//                        kMoneyShotTolerance));
    }

    public void monitorTemperatures() {
        boolean climberStatus = monitorTemp(climber.getPivotTemp(), 70, "Pivot Overheating") &&
                monitorTemp(climber.getExtensionTemp(), 70, "Extension Overheating");

        boolean shooterStatus = monitorTemp(shooter.getMasterTemp(), 70, "Right Shooter Overheating")
                && monitorTemp(shooter.getSlaveTemp(), 70, "Left Shooter Overheating");

        boolean drivetrainStatus = monitorTemp(drivetrain.getLeftFrontDriveTemp(), 70, "Left Front Talon Overheating")
                && monitorTemp(drivetrain.getLeftBackDriveTemp(), 70, "Left Back Talon Overheating")
                && monitorTemp(drivetrain.getRightFrontDriveTemp(), 70, "Right Front Talon Overheating")
                && monitorTemp(drivetrain.getRightBackDriveTemp(), 70, "Right Back Talon Overheating")
                && monitorTemp(drivetrain.getLeftFrontTurnTemp(), 60, "Left Front NEO Overheating")
                && monitorTemp(drivetrain.getLeftBackTurnTemp(), 60, "Left Back NEO Overheating")
                && monitorTemp(drivetrain.getRightFrontTurnTemp(), 60, "Right Front NEO Overheating")
                && monitorTemp(drivetrain.getRightBackTurnTemp(), 60, "Right Back NEO Overheating");

        SmartDashboard.putBoolean("Climber System Check", climberStatus);
        SmartDashboard.putBoolean("Shooter System Check", shooterStatus);
        SmartDashboard.putBoolean("Drivetrain System Check", drivetrainStatus);
    }

    public AllianceColor getInferredAllianceColor() {
        if (DriverStation.getAlliance() == DriverStation.Alliance.Red) {
            return AllianceColor.RED;
        } else if (DriverStation.getAlliance() == DriverStation.Alliance.Blue) {
            return AllianceColor.BLUE;
        } else {
            return allianceColorChooser.getSelected();
        }
    }

    public double getForward() {
        return -driveGamepad.getLeftY();
    }

    public double getStrafe() {
        return -driveGamepad.getLeftX();
    }

    public double getRotateX() {
        return -driveGamepad.getRightX();
    }

    public double getRotateY() {
        return -driveGamepad.getRightY();
    }

    @Override
    public void periodic() {
        stateMachine.run();

        LiveDashboardHelper.putRobotData(getAllianceSpecificPose());
        handleClimbTime();
    }

    public enum CurrentMode {
        SCORING_MODE, CLIMBING_MODE
    }

    public enum ClimbingTargetRung {
        MID_RUNG, HIGH_RUNG
    }

    public enum AllianceColor {
        RED, BLUE
    }

}
