package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.ClimberConfig;
import frc.robot.config.LimitPair;
import frc.robot.config.Target;
import frc.robot.robotState.climbing.DeployHighBarArms;
import frc.robot.util.EnhancedBoolean;

import static frc.robot.Constants.Climber.kExtensionZeroingPercentOutput;
import static frc.robot.Constants.Climber.kFastExtensionZeroingPercentOutput;
import static frc.robot.RobotContainer.currentRobot;

public class Climber implements SubSubsystem {

    private static final ClimberConfig config = currentRobot.getClimberConfig();

    private final DutyCycleEncoder pivotAngleAbsoluteEncoder = new DutyCycleEncoder(
            config.getPivotAngleAbsoluteEncoderConfig().getChannel());

    private final DigitalInput leftExtensionLowerLimit = new DigitalInput(config.getLeftExtensionLowerLimitChannel());
    private final DigitalInput rightExtensionLowerLimit = new DigitalInput(config.getRightExtensionLowerLimitChannel());

    private final TalonFX pivotController = new TalonFX(config.getPivotControllerMotorConfig().getChannelOrID());
    private final TalonFX extensionController = new TalonFX(config.getExtensionControllerMotorConfig().getChannelOrID());

    private final Solenoid climberLock = new Solenoid(PneumaticsModuleType.REVPH, config.getClimberLockSolenoidChannel());
    private final Solenoid highBarArmsSolenoid = new Solenoid(PneumaticsModuleType.REVPH, config.getHighBarArmsSolenoidChannel());

    private final PeriodicIO periodicIO = new PeriodicIO();

    private final EnhancedBoolean isZeroed = new EnhancedBoolean();

    private final double absoluteEncoderCountsPerRev = config.getPivotAngleAbsoluteEncoderConfig().getDistancePerRotation();

    public Climber() {
        // Duty cycle range from Rev Through Bore Encoder specs
        pivotAngleAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        pivotAngleAbsoluteEncoder.setDistancePerRotation(absoluteEncoderCountsPerRev);

        pivotController.configFactoryDefault(10);
        pivotController.configAllSettings(config.getPivotControllerTalonConfig(), 10);
        pivotController.setInverted(config.getPivotControllerMotorConfig().isInverted());
        pivotController.setSensorPhase(config.getPivotControllerMotorConfig().isInverted());
        pivotController.setNeutralMode(NeutralMode.Brake);
        pivotController.enableVoltageCompensation(true);

        extensionController.configFactoryDefault(10);
        extensionController.configAllSettings(config.getExtensionControllerTalonConfig(), 10);
        extensionController.setInverted(config.getExtensionControllerMotorConfig().isInverted());
        extensionController.setSensorPhase(config.getExtensionControllerMotorConfig().isInverted());
        extensionController.setNeutralMode(NeutralMode.Brake);
        extensionController.enableVoltageCompensation(true);

        configPivotStatusFrames();
        configExtensionStatusFrames();

        periodicIO.pivotNeutralMode = NeutralMode.Brake;
        periodicIO.extensionNeutralMode = NeutralMode.Brake;
    }

    @Override
    public synchronized void zeroSensors() {
        loadPivotVerticalReference();
    }

    @Override
    public synchronized void collectData() {
        periodicIO.hasPivotControllerResetOccurred = pivotController.hasResetOccurred();
        periodicIO.hasExtensionControllerResetOccurred = extensionController.hasResetOccurred();

        // Absolute encoder feedback is non-continuous
        double absoluteEncoderDutyCycle = pivotAngleAbsoluteEncoder.getDistance();

        if (config.getPivotAngleAbsoluteEncoderConfig().isInverted()) {
            periodicIO.pivotAbsoluteEncoderPositionNU = absoluteEncoderCountsPerRev -
                    Math.IEEEremainder(absoluteEncoderDutyCycle,
                            absoluteEncoderCountsPerRev);
        } else {
            periodicIO.pivotAbsoluteEncoderPositionNU = Math.IEEEremainder(absoluteEncoderDutyCycle,
                    absoluteEncoderCountsPerRev);
        }

        double currentTime = Timer.getFPGATimestamp();
        double dt = currentTime - periodicIO.lastCollectDataTime;

        periodicIO.pivotAbsoluteEncoderVelocityNU = (periodicIO.pivotAbsoluteEncoderPositionNU
                - periodicIO.lastPivotAbsoluteEncoderPositionNU) / dt;

        periodicIO.lastCollectDataTime = currentTime;
        periodicIO.lastPivotAbsoluteEncoderPositionNU = periodicIO.pivotAbsoluteEncoderPositionNU;

        periodicIO.pivotIntegratedEncoderPositionNU = pivotController.getSelectedSensorPosition();

        boolean reverse = getPivotAbsoluteEncoderPositionNU() < periodicIO.pivotLimits.getReverseSoftLimitThreshold();
        boolean forward = getPivotAbsoluteEncoderPositionNU() > periodicIO.pivotLimits.getForwardsSoftLimitThreshold();

        periodicIO.pivotReverseSoftLimitBool.set(reverse);
        periodicIO.pivotForwardSoftLimitBool.set(forward);

        periodicIO.isLeftExtensionLowerLimitClosed = !leftExtensionLowerLimit.get();
        periodicIO.isRightExtensionLowerLimitClosed = !rightExtensionLowerLimit.get();

        periodicIO.extensionIntegratedEncoderPosition = extensionController.getSelectedSensorPosition();
    }

    @Override
    public synchronized void outputData() {
        if (periodicIO.hasPivotControllerResetOccurred) {
            configPivotStatusFrames();
        }

        if (periodicIO.hasExtensionControllerResetOccurred) {
            configExtensionStatusFrames();
        }

        // When soft limits are rising, enable limiting based feedback from the integrated encoder
        // When soft limits are falling, disable soft limits based on feedback from the integrated encoder

        if (periodicIO.pivotReverseSoftLimitBool.isRisingEdge()) {
            pivotController.configReverseSoftLimitThreshold(periodicIO.pivotIntegratedEncoderPositionNU);
        }

        pivotController.configReverseSoftLimitEnable(periodicIO.pivotReverseSoftLimitBool.get());

        if (periodicIO.pivotForwardSoftLimitBool.isRisingEdge()) {
            pivotController.configForwardSoftLimitThreshold(periodicIO.pivotIntegratedEncoderPositionNU);
        }

        pivotController.configForwardSoftLimitEnable(periodicIO.pivotForwardSoftLimitBool.get());

        if (periodicIO.resetExtensionLimits) {
            System.out.println("Lower limit: " + periodicIO.extensionLimits.getReverseSoftLimitThreshold());
            extensionController.configReverseSoftLimitThreshold(periodicIO.extensionLimits.getReverseSoftLimitThreshold());
            extensionController.configForwardSoftLimitThreshold(periodicIO.extensionLimits.getForwardsSoftLimitThreshold());
            periodicIO.resetExtensionLimits = false;
        }

        extensionController.configReverseSoftLimitEnable(!periodicIO.releaseExtensionLowerLimit);

        if (periodicIO.resetPivotNeutralMode) {
            pivotController.setNeutralMode(periodicIO.pivotNeutralMode);
            periodicIO.resetPivotNeutralMode = false;
        }

        if (periodicIO.resetExtensionNeutralMode) {
            extensionController.setNeutralMode(periodicIO.extensionNeutralMode);
            periodicIO.resetExtensionNeutralMode = false;
        }

        switch (periodicIO.pivotControlState) {
            case ZEROING:
                break;
            case AUTO:
                pivotController.set(ControlMode.MotionMagic, periodicIO.pivotPositionDemandNU);
                periodicIO.pivotPercentOutputDemand = 0;
                break;
            case OPEN_LOOP:
                pivotController.set(ControlMode.PercentOutput, periodicIO.pivotPercentOutputDemand);

                periodicIO.pivotPositionDemandNU = periodicIO.pivotIntegratedEncoderPositionNU;
                break;
            case DISABLED:
                pivotController.set(ControlMode.Disabled, 0);
                periodicIO.pivotPercentOutputDemand = 0;
                periodicIO.pivotPositionDemandNU = 0;
                break;
        }

        switch (periodicIO.extensionControlState) {
            case ZEROING:
                if (isFastZeroing()) {
                    extensionController.set(ControlMode.PercentOutput, kFastExtensionZeroingPercentOutput);
                } else {
                    extensionController.set(ControlMode.PercentOutput, kExtensionZeroingPercentOutput);
                }

                if (isZeroRising()) {
                    extensionController.setSelectedSensorPosition(0);
                }

                periodicIO.extensionPositionDemandNU = 0;

                break;
            case AUTO:
                extensionController.set(ControlMode.MotionMagic, periodicIO.extensionPositionDemandNU);
                periodicIO.extensionPercentOutputDemand = 0;
                break;
            case OPEN_LOOP:
                extensionController.set(ControlMode.PercentOutput, periodicIO.extensionPercentOutputDemand);
                periodicIO.extensionPositionDemandNU = periodicIO.extensionIntegratedEncoderPosition;
                break;
            case DISABLED:
                extensionController.set(ControlMode.Disabled, 0);
                periodicIO.extensionPercentOutputDemand = 0;
                periodicIO.extensionPositionDemandNU = 0;
                break;
        }

        climberLock.set(periodicIO.climberLockStateDemand);
        highBarArmsSolenoid.set(periodicIO.highBarArmsStateDemand);
    }

    @Override
    public void updateShuffleboard() {
        SmartDashboard.putString("Climber/Periodic IO/Pivot Control State", periodicIO.pivotControlState.name());
        SmartDashboard.putString("Climber/Periodic IO/Extension Control State", periodicIO.extensionControlState.name());
        SmartDashboard.putNumber("Climber/Periodic IO/Pivot Percent Output Demand", periodicIO.pivotPercentOutputDemand);
        SmartDashboard.putNumber("Climber/Periodic IO/Pivot Feed Forward", periodicIO.pivotFeedForward);
        SmartDashboard.putNumber("Climber/Periodic IO/Pivot Position Demand NU", periodicIO.pivotPositionDemandNU);
        SmartDashboard.putNumber("Climber/Periodic IO/Extension Percent Output Demand", periodicIO.extensionPercentOutputDemand);
        SmartDashboard.putNumber("Climber/Periodic IO/Extension Position Demand NU", periodicIO.extensionPositionDemandNU);
        SmartDashboard.putBoolean("Climber/Periodic IO/Climber Lock State Demand", periodicIO.climberLockStateDemand);
        SmartDashboard.putBoolean("Climber/Periodic IO/Climber Disc Brake State Demand", periodicIO.climberDiscBrakeStateDemand);
        SmartDashboard.putNumber("Climber/Periodic IO/Pivot Absolute Encoder Position NU", periodicIO.pivotAbsoluteEncoderPositionNU);
        SmartDashboard.putNumber("Climber/Periodic IO/Pivot Absolute Encoder Velocity NU", periodicIO.pivotAbsoluteEncoderPositionNU);
        SmartDashboard.putNumber("Climber/Periodic IO/Pivot Integrated Encoder Position NU", periodicIO.pivotIntegratedEncoderPositionNU);
        SmartDashboard.putBoolean("Climber/Periodic IO/Pivot Reverse Limit", periodicIO.pivotReverseSoftLimitBool.get());
        SmartDashboard.putBoolean("Climber/Periodic IO/Pivot Forward Limit", periodicIO.pivotForwardSoftLimitBool.get());
        SmartDashboard.putBoolean("Climber/Periodic IO/Is Left Extension Lower Limit Closed", periodicIO.isLeftExtensionLowerLimitClosed);
        SmartDashboard.putBoolean("Climber/Periodic IO/Is Right Extension Lower Limit Closed", periodicIO.isRightExtensionLowerLimitClosed);
        SmartDashboard.putNumber("Climber/Periodic IO/Extension Integrated Encoder Position", periodicIO.extensionIntegratedEncoderPosition);

//        SmartDashboard.putNumber(kClimberPivotAngleFromVerticalKey, getPivotAngleFromVertical().getDegrees());
//        SmartDashboard.putNumber(kClimberPivotAngleFromHorizontalKey, getPivotAngleFromHorizontal().getDegrees());

    }

    public void loadPivotVerticalReference() {
        double offsetAbsoluteCounts = getPivotAbsoluteEncoderPositionNU() - config.getVerticalReferenceAbsoluteCounts();
        double setpointIntegratedCounts = offsetAbsoluteCounts * config.getAbsoluteCountsToIntegratedCountsFactor();

        pivotController.setSelectedSensorPosition(setpointIntegratedCounts);
        setPivotPositionDemandNU(setpointIntegratedCounts);
    }

    public boolean isZeroed() {
        return isZeroed.get();
    }

    public void setZeroed(boolean zeroed) {
        isZeroed.set(zeroed);
    }

    public boolean isZeroRising() {
        return isZeroed.isRisingEdge();
    }

    public double getPivotTemp() {
        return pivotController.getTemperature();
    }

    public double getExtensionTemp() {
        return extensionController.getTemperature();
    }

    public ClimberControlState getPivotControlState() {
        return periodicIO.pivotControlState;
    }

    public void setPivotControlState(ClimberControlState pivotControlState) {
        periodicIO.pivotControlState = pivotControlState;
    }

    public ClimberControlState getExtensionControlState() {
        return periodicIO.extensionControlState;
    }

    public void setExtensionControlState(ClimberControlState extensionControlState) {
        periodicIO.extensionControlState = extensionControlState;
    }

    public void setPivotLimits(ClimberPivotLimits limits) {
        setPivotLimits(config.getClimberPivotLimits().get(limits));
    }

    public void setPivotLimits(LimitPair limits) {
        // Only reset limits if they are different from the current limits
        if (!limits.equals(periodicIO.pivotLimits)) {
            periodicIO.pivotLimits = limits;
        }
    }

    public void setExtensionLimits(ClimberExtensionLimits limits) {
        setExtensionLimits(config.getClimberExtensionLimits().get(limits));
    }

    public void setExtensionLimits(LimitPair limits) {
        // Only reset limits if they are different from the current limits
        if (!limits.equals(periodicIO.extensionLimits)) {
            periodicIO.resetExtensionLimits = true;
            periodicIO.extensionLimits = limits;
        }
    }

    public void enableExtensionLowerLimit() {
        periodicIO.releaseExtensionLowerLimit = false;
    }

    public void releaseExtensionLowerLimit() {
        periodicIO.releaseExtensionLowerLimit = true;
    }

    public boolean isFastZeroing() {
        return periodicIO.isFastZeroing;
    }

    public void setFastZeroing(boolean fastZeroing) {
        periodicIO.isFastZeroing = fastZeroing;
    }

    public void setPivotNeutralMode(NeutralMode neutralModeDemand) {
        if (periodicIO.pivotNeutralMode != neutralModeDemand) {
            periodicIO.pivotNeutralMode = neutralModeDemand;
            periodicIO.resetPivotNeutralMode = true;
        }
    }

    public void setExtensionNeutralMode(NeutralMode neutralModeDemand) {
        if (periodicIO.extensionNeutralMode != neutralModeDemand) {
            periodicIO.extensionNeutralMode = neutralModeDemand;
            periodicIO.resetExtensionNeutralMode = true;
        }
    }

    public double getPivotPercentOutputDemand() {
        return periodicIO.pivotPercentOutputDemand;
    }

    public void setPivotPercentOutputDemand(double pivotPercentOutputDemand) {
        periodicIO.pivotPercentOutputDemand = pivotPercentOutputDemand;
    }

    public double getPivotPositionDemandNU() {
        return periodicIO.pivotPositionDemandNU;
    }

    public void setPivotPositionDemandNU(double pivotPositionDemandNU) {
        setPivotPositionDemandNU(pivotPositionDemandNU, 0);
    }

    public void setPivotPositionDemand(Target target) {
        setPivotPositionDemandNU(target.getTarget());
    }

    public void setPivotPositionDemand(ClimberPivotPosition position) {
        setPivotPositionDemandNU(config.getClimberPivotTargets().get(position).getTarget());
    }

    public void setPivotPositionDemand(ClimberPivotPosition position, double feedforward) {
        setPivotPositionDemandNU(config.getClimberPivotTargets().get(position).getTarget(), feedforward);
    }

    public void setPivotPositionDemandNU(double pivotPositionDemandNU, double feedForward) {
        // Reset pivot controller upon new position demand
//        if (pivotPositionDemandNU != periodicIO.pivotPositionDemandNU) {
//            config.getPivotProfiledController().reset(getPivotAbsoluteEncoderPositionNU(),
//                    getPivotAbsoluteEncoderVelocityNU());
//        }

        periodicIO.pivotPositionDemandNU = pivotPositionDemandNU;
        periodicIO.pivotFeedForward = feedForward;
    }

    public double getExtensionPercentOutputDemand() {
        return periodicIO.extensionPercentOutputDemand;
    }

    public void setExtensionPercentOutputDemand(double extensionPercentOutputDemand) {
        periodicIO.extensionPercentOutputDemand = extensionPercentOutputDemand;
    }

    public double getExtensionPositionDemandNU() {
        return periodicIO.extensionPositionDemandNU;
    }

    public void setExtensionPositionDemandNU(double extensionPositionDemandNU) {
        periodicIO.extensionPositionDemandNU = extensionPositionDemandNU;
    }

    public void setExtensionPositionDemand(Target target) {
        setExtensionPositionDemandNU(target.getTarget());
    }

    public void setExtensionPositionDemand(ClimberExtensionPosition position) {
        setExtensionPositionDemandNU(config.getClimberExtensionTargets().get(position).getTarget());
    }

    public boolean isClimberLockUnengaged() {
        return periodicIO.climberLockStateDemand;
    }

    public void setClimberLockStateDemand(boolean unengaged) {
        periodicIO.climberLockStateDemand = unengaged;
    }

    public void toggleClimberLock() {
        setClimberLockStateDemand(!isClimberLockUnengaged());
    }

    public boolean isClimberDiscBrakeUnengaged() {
        return periodicIO.climberDiscBrakeStateDemand;
    }

    public void setClimberDiscBrakeStateDemand(boolean unengaged) {
        periodicIO.climberDiscBrakeStateDemand = unengaged;
    }

    public boolean areHighBarArmsDeployed() {
        return periodicIO.highBarArmsStateDemand;
    }

    public void setHighBarArmsDeployed(boolean deployed) {
        periodicIO.highBarArmsStateDemand = deployed;
    }

    public double getPivotAbsoluteEncoderPositionNU() {
        return periodicIO.pivotAbsoluteEncoderPositionNU;
    }

    public double getPivotAbsoluteEncoderVelocityNU() {
        return periodicIO.pivotAbsoluteEncoderVelocityNU;
    }

    public double getPivotIntegratedEncoderPositionNU() {
        return periodicIO.pivotIntegratedEncoderPositionNU;
    }

    public boolean isLeftExtensionLowerLimitClosed() {
        return periodicIO.isLeftExtensionLowerLimitClosed;
    }

    public boolean isRightExtensionLowerLimitClosed() {
        return periodicIO.isRightExtensionLowerLimitClosed;
    }

    public boolean isLeftExtensionReadyForHigh(){
        return (periodicIO.extensionIntegratedEncoderPosition
                <= config.getClimberExtensionLimits().get(ClimberExtensionLimits.DEPLOY_HIGH_BAR_CLIMB).getForwardsSoftLimitThreshold()
        && periodicIO.extensionIntegratedEncoderPosition
                >= config.getClimberExtensionLimits().get(ClimberExtensionLimits.DEPLOY_HIGH_BAR_CLIMB).getReverseSoftLimitThreshold())
                ;
    }

    public boolean isRightExtensionReadyForHigh(){
        return isLeftExtensionReadyForHigh();
    }

    public double getExtensionIntegratedEncoderPosition() {
        return periodicIO.extensionIntegratedEncoderPosition;
    }

    public Rotation2d getPivotAngleFromVertical() {
        double pivotOffset = getPivotAbsoluteEncoderPositionNU() - config.getVerticalReferenceAbsoluteCounts();
        double rotations = pivotOffset / pivotAngleAbsoluteEncoder.getDistancePerRotation();

        return Rotation2d.fromDegrees(rotations * 360.0);
    }

    // Pivot arm angle from negative x-axis
    public Rotation2d getPivotAngleFromHorizontal() {
        return Rotation2d.fromDegrees(90.0).minus(getPivotAngleFromVertical());
    }

    // Robot pitch angle is CCW positive (in-phase with climber)
    public double getCalculatedFeedForward(Rotation2d robotPitchAngle) {
        Rotation2d robotAngleFromGlobalHorizontal = getPivotAngleFromHorizontal().minus(robotPitchAngle);

        // If pivot arm has rotated CW past the robot vertical
        if (robotAngleFromGlobalHorizontal.getDegrees() > 90.0) {
            robotAngleFromGlobalHorizontal = Rotation2d.fromDegrees(180.0).minus(robotAngleFromGlobalHorizontal);

            double cosineScalar = Math.abs(robotAngleFromGlobalHorizontal.getCos());
            return config.getMaxGravityFeedForward() * cosineScalar;
        }

        double cosineScalar = Math.abs(robotAngleFromGlobalHorizontal.getCos());
        return config.getMaxGravityFeedForward() * -cosineScalar;
    }

    public ClimberConfig getConfig() {
        return config;
    }

    private void configPivotStatusFrames() {
        pivotController.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        pivotController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        pivotController.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 1000);
        pivotController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 1000);
        pivotController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
        pivotController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 1000);
        pivotController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
        pivotController.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);
        pivotController.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000);
    }

    private void configExtensionStatusFrames() {
        extensionController.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        extensionController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
        extensionController.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 1000);
        extensionController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 10);
        extensionController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
        extensionController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 100);
        extensionController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
        extensionController.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);
        extensionController.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000);
    }

    public void configExtensionSmartMotion(double cruiseVelocity, double maxAcceleration) {
        extensionController.configMotionCruiseVelocity(cruiseVelocity);
        extensionController.configMotionAcceleration(maxAcceleration);
    }

    public void configPivotSmartMotion(double cruiseVelocity, double maxAcceleration) {
        pivotController.configMotionCruiseVelocity(cruiseVelocity);
        pivotController.configMotionAcceleration(maxAcceleration);
    }

    public enum ClimberControlState {
        ZEROING, AUTO, OPEN_LOOP, DISABLED
    }

    public enum ClimberExtensionPosition {
        STOWED_HEIGHT,
        CLOSE_IN_TO_ZERO_LENGTH,
        PULL_UP_TO_HOOK_ONTO_MID_BAR_LENGTH,
        MID_BAR_CLIMB_LINING_UP_TO_MID_BAR_LENGTH,
        FINALIZE_HIGH_BAR_CLIMB_LENGTH,
//        LENGTH_TO_DISENGAGE_FROM_MID_BAR,
//        HOOKING_ONTO_HIGH_BAR_LENGTH,
//        PULLING_UP_TO_HIGH_BAR_TRANSFER_LENGTH,
//        LENGTH_TO_DISENGAGE_FROM_HIGH_BAR,
//        HOOKING_ONTO_TRAVERSAL_BAR_LENGTH,
//        LENGTH_TO_HANG_FROM_TRAVERSAL_BAR
    }

    public enum ClimberExtensionLimits {
        STOWED, // Corresponds to STOWED_HEIGHT
        EXTENSION_FULL_ROM,
        MID_BAR_FINALIZE_CLIMB, // Corresponds to PULL_UP_TO_HOOK_ONTO_MID_BAR_LENGTH
        DEPLOY_HIGH_BAR_CLIMB // Corresponds to DEPLOY_HIGH_BAR_CLIMB
//        HIGH_BAR_TRANSFER_TO_FIXED_ARM, // Corresponds to PULLING_UP_TO_HIGH_BAR_TRANSFER_LENGTH
    }

    public enum ClimberPivotPosition {
        // If the position begins with ANGLE, rotation is CW
        // If the position ends with ANGLE, rotation is CCW

        STOWED_ANGLE,
        FINALIZE_HIGH_CLIMB_ANGLE
//        ANGLE_HOOK_THETA_FOR_MID_BAR,
//        PIVOT_BACK_TO_TRANSFER,
//        REACHING_FOR_HIGH_BAR_PIVOT_ANGLE,
//        ANGLE_TO_HOOK_ONTO_HIGH_BAR,
//        ANGLE_TO_POSITION_FIXED_ARM_FOR_HIGH_BAR_TRANSFER,
//        FIXED_ARM_TO_HOOK_ONTO_HIGH_BAR_ANGLE,
//        REACHING_FOR_TRAVERSAL_BAR_PIVOT_ANGLE,
//        ANGLE_TO_HOOK_ONTO_TRAVERSAL_BAR
    }

    public enum ClimberPivotLimits {
        PIVOT_STOWED, // Corresponds to STOWED_ANGLE
        PIVOT_FULL_ROM,
//        PIVOT_PULL_UP_TO_MID_BAR, // Corresponds to ANGLE_HOOK_THETA_FOR_MID_BAR
//        PIVOT_PULL_UP_TO_HIGH_BAR, // Corresponds to ANGLE_TO_HOOK_ONTO_HIGH_BAR
//        PIVOT_PULL_UP_TO_TRANSFER_HIGH_BAR, // Corresponds to ANGLE_TO_POSITION_FIXED_ARM_FOR_HIGH_BAR_TRANSFER
//        PIVOT_PULL_UP_TO_TRAVERSAL_BAR, // Corresponds to ANGLE_TO_HOOK_ONTO_TRAVERSAL_BAR
    }

    public static class PeriodicIO {
        // Inputs
        public boolean hasPivotControllerResetOccurred;
        public boolean hasExtensionControllerResetOccurred;
        public double pivotAbsoluteEncoderPositionNU;
        private double lastCollectDataTime;
        private double lastPivotAbsoluteEncoderPositionNU;
        public double pivotAbsoluteEncoderVelocityNU;
        public double pivotIntegratedEncoderPositionNU;
        public EnhancedBoolean pivotReverseSoftLimitBool = new EnhancedBoolean();
        public EnhancedBoolean pivotForwardSoftLimitBool = new EnhancedBoolean();
        public boolean isLeftExtensionLowerLimitClosed;
        public boolean isRightExtensionLowerLimitClosed;
        public double extensionIntegratedEncoderPosition;
        // Outputs
        public LimitPair pivotLimits = config.getClimberPivotLimits().get(ClimberPivotLimits.PIVOT_STOWED);
        public boolean resetExtensionLimits;
        public LimitPair extensionLimits = config.getClimberExtensionLimits().get(ClimberExtensionLimits.STOWED);
        public boolean releaseExtensionLowerLimit;
        public double pivotPercentOutputDemand;
        public double pivotFeedForward;
        public double pivotPositionDemandNU;
        public double extensionPercentOutputDemand;
        public double extensionPositionDemandNU;
        public boolean climberLockStateDemand;
        public boolean climberDiscBrakeStateDemand;
        public boolean highBarArmsStateDemand;
        public ClimberControlState pivotControlState = ClimberControlState.DISABLED;
        public ClimberControlState extensionControlState = ClimberControlState.DISABLED;
        public boolean isFastZeroing;
        private boolean resetPivotNeutralMode;
        private NeutralMode pivotNeutralMode;
        private boolean resetExtensionNeutralMode;
        private NeutralMode extensionNeutralMode;
    }

}
