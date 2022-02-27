package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.OI;
import frc.robot.robotState.Disabled;
import frc.robot.stateMachine.StateMachine;
import frc.robot.util.UtilMethods;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DriverPreferences.kExtensionManualOverrideDeadband;
import static frc.robot.Constants.DriverPreferences.kPivotManualOverrideDeadband;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.OI.dangerButton;
import static frc.robot.OI.manipulationGamepad;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class Superstructure extends SubsystemBase {

    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();

    private boolean isEnabled = false;
    private CurrentMode currentMode = CurrentMode.SCORING_MODE;

    private double currentTargetFlywheelVelocity = 0;

    private boolean isInAuton = false;
    private boolean doesAutonNeedToIntake = false;
    private boolean doesAutonNeedToShoot = false;
    private boolean doesAutonNeedToAlignAndShoot = false;

    private final StateMachine stateMachine;

    public Superstructure() {
        stateMachine = new StateMachine("Superstructure", new Disabled());
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

    public void handleTransportConveyorManualOverride() {
        if (OI.overrideTransportConveyorButton.get()) {
            godSubsystem.getConveyor().setTransportDemand(conveyor.getConfig().getTransportIntakePercentOutput());
        } else {
            godSubsystem.getConveyor().setTransportDemand(0);
        }
    }

    public void handleFeedConveyorManualOverride() {
        if (OI.overrideFeedConveyorButton.get()) {
            godSubsystem.getConveyor().setFeedDemand(conveyor.getConfig().getFeedShootPercentOutput());
        } else {
            godSubsystem.getConveyor().setFeedDemand(0);
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

            double pivotJoystick = -manipulationGamepad.getLeftX();

            pivotJoystick = UtilMethods.limitMagnitude(pivotJoystick,
                    climber.getConfig().getPivotManualPercentOutputLimit());

            climber.setPivotPercentOutputDemand(pivotJoystick);
        } else {
            climber.setPivotControlState(Climber.ClimberControlState.AUTO);
        }
    }

    public void handleExtensionManualOverride() {
        if (isExtensionManualOverride()) {
            climber.setExtensionControlState(Climber.ClimberControlState.OPEN_LOOP);

            double extensionJoystick = -manipulationGamepad.getRightY();

            extensionJoystick = UtilMethods.limitMagnitude(extensionJoystick,
                    climber.getConfig().getExtensionManualPercentOutputLimit());

            climber.setExtensionPercentOutputDemand(extensionJoystick);
        } else {
            climber.setExtensionControlState(Climber.ClimberControlState.AUTO);
        }
    }

    @Override
    public void periodic() {
        stateMachine.run();

        SmartDashboard.putData(kDrivetrainPeriodicIOKey, drivetrain.getPeriodicIOSendable());
        SmartDashboard.putData(kIntakePeriodicIOKey, intake.getPeriodicIOSendable());
        SmartDashboard.putData(kConveyorPeriodicIOKey, conveyor.getPeriodicIOSendable());
        SmartDashboard.putData(kShooterPeriodicIOKey, shooter.getPeriodicIOSendable());
        SmartDashboard.putData(kClimberPeriodicIOKey, climber.getPeriodicIOSendable());

        SmartDashboard.putNumber(kShooterCurrentTargetVelocity, getCurrentTargetFlywheelVelocity());
        SmartDashboard.putNumber(kLimelightDistanceFeetKey, LimelightHelper.getDistanceToTargetFeet());

        SmartDashboard.putNumber(kClimberPivotAngleFromVertical, climber.getPivotAngleFromVertical().getDegrees());
        SmartDashboard.putNumber(kClimberPivotAngleFromHorizontal, climber.getPivotAngleFromHorizontal().getDegrees());
    }

    public enum CurrentMode {
        SCORING_MODE, CLIMBING_MODE
    }

}
