package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.ShooterConfig;
import frc.robot.util.UtilMethods;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.vision.LimelightHelper;

import java.util.logging.Level;

import static frc.robot.Constants.PIDProfileSlots.kShootingIndex;
import static frc.robot.Constants.PIDProfileSlots.kSpinningUpIndex;
import static frc.robot.Constants.Shooter.*;
import static frc.robot.Constants.SmartDashboardKeys.kShooterBallQualityAdditive;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.robotLogger;

public class Shooter implements SubSubsystem {

    private final ShooterConfig config = currentRobot.getShooterConfig();

    private final TalonFX flywheelMasterController = new TalonFX(
            config.getFlywheelMasterControllerMotorConfig().getChannelOrID());
    private final TalonFX flywheelSlaveController = new TalonFX(
            config.getFlywheelSlaveControllerMotorConfig().getChannelOrID());

    private final Servo adjustableHoodServo = new Servo(
            config.getAdjustableHoodServoConfig().getChannelOrID());

    private final PeriodicIO periodicIO = new PeriodicIO();

    public Shooter() {
//        flywheelMasterController.configFactoryDefault(10);
        flywheelMasterController.configAllSettings(config.getFlywheelMasterControllerTalonConfig(), 10);
        flywheelMasterController.setInverted(config.getFlywheelMasterControllerMotorConfig().isInverted());
        flywheelMasterController.setSensorPhase(config.getFlywheelMasterControllerMotorConfig().isInverted());
        flywheelMasterController.setNeutralMode(NeutralMode.Coast);
        flywheelMasterController.configVoltageCompSaturation(12.0);
        flywheelMasterController.enableVoltageCompensation(true);

//        flywheelSlaveController.configFactoryDefault(10);
        flywheelSlaveController.configAllSettings(config.getFlywheelSlaveControllerTalonConfig(), 10);
        flywheelSlaveController.setInverted(TalonFXInvertType.OpposeMaster);
        flywheelSlaveController.setSensorPhase(config.getFlywheelSlaveControllerMotorConfig().isInverted());
        flywheelSlaveController.setNeutralMode(NeutralMode.Coast);
        flywheelSlaveController.enableVoltageCompensation(false);
        configFollower();

        configFlywheelMasterStatusFrames();
        configFlywheelSlaveStatusFrames();

        // From L16-R datasheet
        adjustableHoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);

        periodicIO.adjustableHoodDutyCycleDemand = kDefaultHoodAngle;
        periodicIO.lastAdjustableHoodDutyCycleDemand = kDefaultHoodAngle;
        periodicIO.estimatedHoodPosition = kDefaultHoodAngle;
    }

    @Override
    public synchronized void zeroSensors() {

    }

    @Override
    public synchronized void collectData() {
        periodicIO.flywheelVelocityNU = flywheelMasterController.getSelectedSensorVelocity();
//        periodicIO.flywheelClosedLoopErrorNU = flywheelMasterController.getClosedLoopError();
        LimelightHelper.updateData();

        double demand = UtilMethods.limitRange(periodicIO.adjustableHoodDutyCycleDemand, kHoodLowerLimit, 1.0);
        double savedPosition = UtilMethods.limitRange(periodicIO.savedLastPosition, kHoodLowerLimit, 1.0);

        double displacement = demand - savedPosition;
        double timeToMove = (kHoodTransitionTimeSeconds / kFullHoodAngleRange) *
                Math.abs(displacement);
        double currentTime = Timer.getFPGATimestamp();

//        SmartDashboard.putNumber("Displacement", displacement);
//        SmartDashboard.putNumber("Time to move", timeToMove);

        if (currentTime - periodicIO.lastAdjustableHoodChangeFPGATime < timeToMove) {
            // Hood is in motion
            double motionSign = Math.signum(displacement);
            double dx = motionSign * (currentTime - periodicIO.lastAdjustableHoodChangeFPGATime)
                    * (kFullHoodAngleRange / kHoodTransitionTimeSeconds);

            periodicIO.estimatedHoodPosition += dx;
            periodicIO.isHoodReady = false;
        } else {
            // Hood has reached setpoint
            periodicIO.estimatedHoodPosition = periodicIO.adjustableHoodDutyCycleDemand;
            periodicIO.isHoodReady = true;
        }
    }

    @Override
    public synchronized void outputData() {
        int masterID = config.getFlywheelMasterControllerMotorConfig().getChannelOrID();

        if (periodicIO.hasFlywheelMasterControllerResetOccurred) {
            configFlywheelMasterStatusFrames();
        }

        if (periodicIO.hasFlywheelSlaveControllerResetOccurred) {
            configFlywheelSlaveStatusFrames();
        }

        if (periodicIO.resetSelectedProfileSlot) {
            flywheelMasterController.selectProfileSlot(periodicIO.selectedProfileSlot.index, 0);
            periodicIO.resetSelectedProfileSlot = false;
        }

        switch (periodicIO.shooterControlState) {
            case VELOCITY:
                flywheelMasterController.set(ControlMode.Velocity, periodicIO.flywheelDemand);
                flywheelSlaveController.follow(flywheelMasterController);
//                flywheelSlaveController.set(TalonFXControlMode.Follower, masterID);
                break;
            case OPEN_LOOP:
                flywheelMasterController.set(ControlMode.PercentOutput, periodicIO.flywheelDemand);
                flywheelSlaveController.follow(flywheelMasterController);
//                flywheelSlaveController.set(TalonFXControlMode.Follower, masterID);
                break;
            case DISABLED:
                flywheelMasterController.set(ControlMode.Disabled, 0.0);
                flywheelSlaveController.follow(flywheelMasterController);
//                flywheelSlaveController.set(TalonFXControlMode.Follower, masterID);
                break;
        }

        double currentFPGATime = Timer.getFPGATimestamp();

        if (periodicIO.lastAdjustableHoodDutyCycleDemand != periodicIO.adjustableHoodDutyCycleDemand) {
            periodicIO.lastAdjustableHoodChangeFPGATime = currentFPGATime;
            periodicIO.savedLastPosition = periodicIO.lastAdjustableHoodDutyCycleDemand;
            periodicIO.lastAdjustableHoodDutyCycleDemand = periodicIO.adjustableHoodDutyCycleDemand;
        }

        adjustableHoodServo.setSpeed(
                UtilMethods.limitRange(periodicIO.adjustableHoodDutyCycleDemand, kHoodLowerLimit, 1.0));
    }

    @Override
    public void updateShuffleboard() {
        SmartDashboard.putString("Shooter/Periodic IO/Aim Target", periodicIO.aimTarget.name());
        SmartDashboard.putString("Shooter/Periodic IO/Shooter Control State", periodicIO.shooterControlState.name());
        SmartDashboard.putString("Shooter/Periodic IO/Selected Profile Slot", periodicIO.selectedProfileSlot.name());
        SmartDashboard.putNumber("Shooter/Periodic IO/Flywheel Demand", periodicIO.flywheelDemand);
        SmartDashboard.putNumber("Shooter/Periodic IO/Adjustable Hood Demand", periodicIO.adjustableHoodDutyCycleDemand);
        SmartDashboard.putNumber("Shooter/Periodic IO/Flywheel Velocity NU", periodicIO.flywheelVelocityNU);
//        SmartDashboard.putNumber("Shooter/Periodic IO/Flywheel Closed Loop Error NU", periodicIO.flywheelClosedLoopErrorNU);
        SmartDashboard.putNumber("Shooter/Periodic IO/Estimated Hood Position", periodicIO.estimatedHoodPosition);
        SmartDashboard.putBoolean("Shooter/Periodic IO/Is Hood Ready", periodicIO.isHoodReady);
    }

    public void configFollower() {
        flywheelSlaveController.follow(flywheelMasterController);
    }
    public AimTarget getAimTarget() {
        return periodicIO.aimTarget;
    }

    public void setAimTarget(AimTarget target) {
        periodicIO.aimTarget = target;
    }

    public ShooterControlState getShooterControlState() {
        return periodicIO.shooterControlState;
    }

    public void setShooterControlState(ShooterControlState shooterControlState) {
        periodicIO.shooterControlState = shooterControlState;
    }

    public ShooterProfileSlot getSelectedProfileSlot() {
        return periodicIO.selectedProfileSlot;
    }

    public void setSelectedProfileSlot(ShooterProfileSlot selectedProfileSlot) {
        if (periodicIO.selectedProfileSlot != selectedProfileSlot) {
            periodicIO.selectedProfileSlot = selectedProfileSlot;
            periodicIO.resetSelectedProfileSlot = true;
        }
    }

    public double getFlywheelDemand() {
        return periodicIO.flywheelDemand;
    }

    public void setFlywheelDemand(double flywheelDemand) {
        periodicIO.flywheelDemand = flywheelDemand;
    }

    public double getAdjustableHoodDutyCycleDemand() {
        return periodicIO.adjustableHoodDutyCycleDemand;
    }

    public void setAdjustableHoodDutyCycleDemand(double adjustableHoodDutyCycleDemand) {
        periodicIO.adjustableHoodDutyCycleDemand = adjustableHoodDutyCycleDemand;
    }

    public double getLastAdjustableHoodChangeFPGATime() {
        return periodicIO.lastAdjustableHoodChangeFPGATime;
    }

    public double getLastAdjustableHoodDutyCycleDemand() {
        return periodicIO.lastAdjustableHoodDutyCycleDemand;
    }

    public double getFlywheelVelocityNU() {
        return periodicIO.flywheelVelocityNU;
    }

//    public double getFlywheelClosedLoopErrorNU() {
//        return periodicIO.flywheelClosedLoopErrorNU;
//    }

    public double getEstimatedHoodPosition() {
        return periodicIO.estimatedHoodPosition;
    }

    public Rotation2d getHoodAngleFromHorizontal() {
        double hoodPosition = periodicIO.estimatedHoodPosition - kHoodLowerLimit;
        return Rotation2d.fromDegrees(kHoodAngleToDegreesIntercept + kHoodAngleToDegreesSlope * hoodPosition);
    }
    
    public boolean isHoodReady() {
        return periodicIO.isHoodReady;
    }

    public double getMasterTemp() {
        return flywheelMasterController.getTemperature();
    }

    public double getSlaveTemp() {
        return flywheelSlaveController.getTemperature();
    }

    public ShooterConfig getConfig() {
        return config;
    }

    public double getEstimatedHoodAngleFromTarget() {
        if (LimelightHelper.getTV() <= 0) {
            robotLogger.log(Level.WARNING, "No target found for shooter. Using default hood angle");

            if (getAimTarget() == AimTarget.HIGH_GOAL) {
                return kDefaultHighGoalHoodAngle;
            } else {
                return kDefaultLowGoalHoodAngle;
            }
        }

        double distanceFeet = LimelightHelper.getDistanceToTargetFeet();
        InterpolatingDouble result;

        result = config.getHoodAngleMap(getAimTarget()).getInterpolated(new InterpolatingDouble(distanceFeet));

        if (result != null) {
            return result.value;
        } else {
            return 0;
        }
    }

    public double getEstimatedVelocityFromTarget() {
        if (LimelightHelper.getTV() <= 0) {
            robotLogger.log(Level.WARNING, "No target found for shooter. Using default flywheel velocity");

            if (getAimTarget() == AimTarget.HIGH_GOAL) {
                return kDefaultHighGoalVelocity;
            } else {
                return kDefaultLowGoalVelocity;
            }
        }

        double distanceFeet = LimelightHelper.getDistanceToTargetFeet();
        robotLogger.log(Level.INFO, "Limelight distance: " + distanceFeet);
        InterpolatingDouble result;

        result = config.getFlywheelVelocityMap(getAimTarget()).getInterpolated(new InterpolatingDouble(distanceFeet));

        double ballQualityAdditive = UtilMethods.limitMagnitude(
                SmartDashboard.getNumber(kShooterBallQualityAdditive, 0.0), 700);

        if (result != null) {
            return UtilMethods.limitMagnitude(result.value + ballQualityAdditive, kAbsoluteMaximumVelocityNU);
        } else {
            return kDefaultVelocityRawUnits;
        }
    }

    private void configFlywheelMasterStatusFrames() {
        flywheelMasterController.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        flywheelMasterController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        flywheelMasterController.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 1000);
        flywheelMasterController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 1000);
        flywheelMasterController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
        flywheelMasterController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 100);
        flywheelMasterController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
        flywheelMasterController.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);
        flywheelMasterController.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000);
    }

    private void configFlywheelSlaveStatusFrames() {
        flywheelSlaveController.setStatusFramePeriod(StatusFrame.Status_1_General, 255);
        flywheelSlaveController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
        flywheelSlaveController.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 200);
        flywheelSlaveController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 1000);
        flywheelSlaveController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
        flywheelSlaveController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 1000);
        flywheelSlaveController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
        flywheelSlaveController.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);
        flywheelSlaveController.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000);
    }

    public enum ShooterControlState {
        VELOCITY, OPEN_LOOP, DISABLED
    }

    public enum ShooterProfileSlot {
        SPINNING_UP_SLOT(kSpinningUpIndex),
        SHOOTING_SLOT(kShootingIndex);

        private final int index;

        ShooterProfileSlot(int idx) {
            index = idx;
        }

        public int getIndex() {
            return index;
        }
    }

    public enum AimTarget {
        LOW_GOAL,
        HIGH_GOAL
    }

    public static class PeriodicIO {
        // Inputs
        public boolean hasFlywheelMasterControllerResetOccurred;
        public boolean hasFlywheelSlaveControllerResetOccurred;
        public double flywheelVelocityNU;
        //        public double flywheelClosedLoopErrorNU;
        public double estimatedHoodPosition;
        public boolean isHoodReady;

        // Outputs
        public AimTarget aimTarget = AimTarget.HIGH_GOAL;
        public ShooterControlState shooterControlState = ShooterControlState.DISABLED;

        public ShooterProfileSlot selectedProfileSlot = ShooterProfileSlot.SPINNING_UP_SLOT;
        public boolean resetSelectedProfileSlot = false;
        public double flywheelDemand;
        public double adjustableHoodDutyCycleDemand;
        public double lastAdjustableHoodDutyCycleDemand;
        public double savedLastPosition;
        public double lastAdjustableHoodChangeFPGATime;
    }

}
