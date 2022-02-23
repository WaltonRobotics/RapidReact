package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.DriverPreferences.kExtensionManualOverrideDeadband;
import static frc.robot.Constants.DriverPreferences.kPivotManualOverrideDeadband;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.OI.dangerButton;
import static frc.robot.OI.manipulationGamepad;

public class Superstructure extends SubsystemBase {

    private final Drivetrain drivetrain = new Drivetrain();
    private final Intake intake = new Intake();
    private final Conveyor conveyor = new Conveyor();
    private final Shooter shooter = new Shooter();
    private final Climber climber = new Climber();

    private boolean isEnabled = false;
    private CurrentMode currentMode = CurrentMode.SCORING_MODE;

    private double currentTargetFlywheelVelocity = 0;

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

    public double getCurrentTime() {
        return Timer.getFPGATimestamp();
    }

    public boolean isPivotManualOverride() {
        return dangerButton.get() && Math.abs(manipulationGamepad.getLeftX()) > kPivotManualOverrideDeadband;
    }

    public boolean isExtensionManualOverride() {
        return dangerButton.get() && Math.abs(manipulationGamepad.getRightY()) > kExtensionManualOverrideDeadband;
    }

    public void handlePivotManualOverride() {
        if (isPivotManualOverride()) {
            climber.setPivotControlState(Climber.ClimberControlState.OPEN_LOOP);

            double pivotJoystick = -manipulationGamepad.getLeftX();

            climber.setPivotPercentOutputDemand(pivotJoystick);
        } else {
            climber.setPivotControlState(Climber.ClimberControlState.AUTO);
        }
    }

    public void handleExtensionManualOverride() {
        if (isExtensionManualOverride()) {
            climber.setExtensionControlState(Climber.ClimberControlState.OPEN_LOOP);

            double extensionJoystick = -manipulationGamepad.getRightY();

            climber.setExtensionPercentOutputDemand(extensionJoystick);
        } else {
            climber.setExtensionControlState(Climber.ClimberControlState.AUTO);
        }
    }

    @Override
    public void periodic() {
        SmartDashboard.putData(kDrivetrainPeriodicIOKey, drivetrain.getPeriodicIOSendable());
        SmartDashboard.putData(kIntakePeriodicIOKey, intake.getPeriodicIOSendable());
        SmartDashboard.putData(kConveyorPeriodicIOKey, conveyor.getPeriodicIOSendable());
        SmartDashboard.putData(kShooterPeriodicIOKey, shooter.getPeriodicIOSendable());
        SmartDashboard.putData(kClimberPeriodicIOKey, climber.getPeriodicIOSendable());

        SmartDashboard.putNumber(kClimberPivotAngleFromVertical, climber.getPivotAngleFromVertical().getDegrees());
        SmartDashboard.putNumber(kClimberPivotAngleFromHorizontal, climber.getPivotAngleFromHorizontal().getDegrees());
    }

    public enum CurrentMode {
        SCORING_MODE, CLIMBING_MODE
    }

}
