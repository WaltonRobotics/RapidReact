package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Solenoid;
import frc.robot.config.ClimberConfig;
import frc.robot.config.LimitPair;
import frc.robot.util.EnhancedBoolean;
import org.apache.commons.math3.geometry.euclidean.threed.Rotation;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static edu.wpi.first.wpilibj.PneumaticsModuleType.CTREPCM;
import static frc.robot.Constants.Climber.kExtensionZeroingPercentOutput;
import static frc.robot.RobotContainer.currentRobot;

public class Climber implements SubSubsystem {

    private final ClimberConfig config = currentRobot.getClimberConfig();

    private final DutyCycleEncoder pivotAngleAbsoluteEncoder = new DutyCycleEncoder(
            config.getPivotAngleAbsoluteEncoderConfig().getChannel());

    private final DigitalInput leftExtensionLowerLimit = new DigitalInput(config.getLeftExtensionLowerLimitChannel());
    private final DigitalInput rightExtensionLowerLimit = new DigitalInput(config.getRightExtensionLowerLimitChannel());

    private final TalonFX pivotController = new TalonFX(config.getPivotControllerMotorConfig().getChannelOrID());
    private final TalonFX extensionController = new TalonFX(config.getExtensionControllerMotorConfig().getChannelOrID());

    private final Solenoid leftClimberLock = new Solenoid(CTREPCM, config.getLeftClimberLockChannel());
    private final Solenoid rightClimberLock = new Solenoid(CTREPCM, config.getRightClimberLockChannel());
    private final DoubleSolenoid climberDiscBrake = new DoubleSolenoid(CTREPCM,
            config.getClimberDiscBrakeForwardChannel(), config.getClimberDiscBrakeReverseChannel());

    private final PeriodicIO periodicIO = new PeriodicIO();

    private final EnhancedBoolean isZeroed = new EnhancedBoolean();

    public Climber() {
        // Duty cycle range from Rev Through Bore Encoder specs
        pivotAngleAbsoluteEncoder.setDutyCycleRange(1.0 / 1025.0, 1024.0 / 1025.0);
        pivotAngleAbsoluteEncoder.setDistancePerRotation(
                config.getPivotAngleAbsoluteEncoderConfig().getDistancePerRotation());

        pivotController.configFactoryDefault(10);
        pivotController.configAllSettings(config.getPivotControllerTalonConfig(), 10);
        pivotController.setInverted(config.getPivotControllerMotorConfig().isInverted());
        pivotController.setSensorPhase(config.getPivotControllerMotorConfig().isInverted());
        pivotController.setNeutralMode(NeutralMode.Brake);
        pivotController.enableVoltageCompensation(true);
        pivotController.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

        extensionController.configFactoryDefault(10);
        extensionController.configAllSettings(config.getExtensionControllerTalonConfig(), 10);
        extensionController.setInverted(config.getExtensionControllerMotorConfig().isInverted());
        extensionController.setSensorPhase(config.getExtensionControllerMotorConfig().isInverted());
        extensionController.setNeutralMode(NeutralMode.Brake);
        extensionController.enableVoltageCompensation(true);
        extensionController.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 10);

        periodicIO.pivotNeutralMode = NeutralMode.Brake;
        periodicIO.extensionNeutralMode = NeutralMode.Brake;
    }

    @Override
    public void zeroSensors() {
        loadPivotVerticalReference();
    }

    @Override
    public void collectData() {
        // Absolute encoder feedback is non-continuous
        periodicIO.pivotAbsoluteEncoderPositionNU = Math.IEEEremainder(pivotAngleAbsoluteEncoder.getDistance(),
                pivotAngleAbsoluteEncoder.getDistancePerRotation());

        periodicIO.pivotIntegratedEncoderPositionNU = pivotController.getSelectedSensorPosition();

        periodicIO.isLeftExtensionLowerLimitClosed = leftExtensionLowerLimit.get();
        periodicIO.isRightExtensionLowerLimitClosed = rightExtensionLowerLimit.get();

        periodicIO.extensionIntegratedEncoderPosition = extensionController.getSelectedSensorPosition();
    }

    @Override
    public void outputData() {
        if (periodicIO.resetPivotLimits) {
            pivotController.configForwardSoftLimitThreshold(1);
            pivotController.configReverseSoftLimitThreshold(0);
            periodicIO.resetPivotLimits = false;
        }

        if (periodicIO.resetExtensionLimits) {
            extensionController.configForwardSoftLimitThreshold(1);
            extensionController.configReverseSoftLimitThreshold(0);
            periodicIO.resetExtensionLimits = false;
        }

        extensionController.configReverseSoftLimitEnable(!periodicIO.releaseExtensionLowerLimit);

        if (periodicIO.resetPivotNeutralMode) {
            pivotController.setNeutralMode(periodicIO.pivotNeutralMode);
            periodicIO.resetPivotNeutralMode = false;
        }

        if (periodicIO.resetExtensionNeutralMode) {
            pivotController.setNeutralMode(periodicIO.extensionNeutralMode);
            periodicIO.resetExtensionNeutralMode = false;
        }

        switch (periodicIO.pivotControlState) {
            case ZEROING:
                break;
            case AUTO:
                pivotController.set(ControlMode.MotionMagic, periodicIO.pivotPositionDemandNU,
                        DemandType.ArbitraryFeedForward, periodicIO.pivotFeedForward);
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
                extensionController.set(ControlMode.PercentOutput, kExtensionZeroingPercentOutput);

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

        leftClimberLock.set(periodicIO.leftClimberLockStateDemand);
        rightClimberLock.set(periodicIO.rightClimberLockStateDemand);
        climberDiscBrake.set(periodicIO.climberDiscBrakeStateDemand ? kForward : kReverse);
    }

    @Override
    public Sendable getPeriodicIOSendable() {
        return periodicIO;
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
        periodicIO.resetPivotLimits = true;
        periodicIO.pivotLimits = limits;
    }

    public void setExtensionLimits(ClimberExtensionLimits limits) {
        setExtensionLimits(config.getClimberExtensionLimits().get(limits));
    }

    public void setExtensionLimits(LimitPair limits) {
        periodicIO.resetExtensionLimits = true;
        periodicIO.extensionLimits = limits;
    }

    public void enableExtensionLowerLimit() {
        periodicIO.releaseExtensionLowerLimit = false;
    }

    public void releaseExtensionLowerLimit() {
        periodicIO.releaseExtensionLowerLimit = true;
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

    public void setPivotPositionDemand(ClimberPivotPosition position) {
        setPivotPositionDemandNU(config.getClimberPivotTargets().get(position).getTarget());
    }

    public void setPivotPositionDemand(ClimberPivotPosition position, double feedforward) {
        setPivotPositionDemandNU(config.getClimberPivotTargets().get(position).getTarget(), feedforward);
    }

    public void setPivotPositionDemandNU(double pivotPositionDemandNU, double feedForward) {
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

    public void setExtensionPositionDemand(ClimberExtensionPosition position) {
        setExtensionPositionDemandNU(config.getClimberExtensionTargets().get(position).getTarget());
    }

    public boolean isLeftClimberLockUnengaged() {
        return periodicIO.leftClimberLockStateDemand;
    }

    public void setLeftClimberLockStateDemand(boolean unengaged) {
        periodicIO.leftClimberLockStateDemand = unengaged;
    }

    public void toggleLeftClimberLock() {
        setLeftClimberLockStateDemand(!isLeftClimberLockUnengaged());
    }

    public boolean isRightClimberLockUnengaged() {
        return periodicIO.rightClimberLockStateDemand;
    }

    public void setRightClimberLockStateDemand(boolean unengaged) {
        periodicIO.rightClimberLockStateDemand = unengaged;
    }

    public void toggleRightClimberLock() {
        setRightClimberLockStateDemand(!isRightClimberLockUnengaged());
    }

    public boolean getClimberDiscBrakeStateDemand() {
        return periodicIO.climberDiscBrakeStateDemand;
    }

    public void setClimberDiscBrakeStateDemand(boolean climberDiscBrakeStateDemand) {
        periodicIO.climberDiscBrakeStateDemand = climberDiscBrakeStateDemand;
    }

    public double getPivotAbsoluteEncoderPositionNU() {
        return periodicIO.pivotAbsoluteEncoderPositionNU;
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

    public double getExtensionIntegratedEncoderPosition() {
        return periodicIO.extensionIntegratedEncoderPosition;
    }

    public Rotation2d getPivotAngleFromVertical() {
        double pivotPosition = getPivotIntegratedEncoderPositionNU();
        double rotations = pivotPosition / config.getIntegratedCountsPerRev();

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

    public enum ClimberControlState {
        ZEROING, AUTO, OPEN_LOOP, DISABLED
    }

    public enum ClimberExtensionPosition {
        STOWED_HEIGHT,
        LINING_UP_TO_MID_BAR_LENGTH,
        PULL_UP_TO_HOOK_ONTO_MID_BAR_LENGTH,
        LENGTH_TO_DISENGAGE_FROM_MID_BAR,
        HOOKING_ONTO_HIGH_BAR_LENGTH,
        PULLING_UP_TO_HIGH_BAR_TRANSFER_LENGTH,
        LENGTH_TO_DISENGAGE_FROM_HIGH_BAR,
        HOOKING_ONTO_TRAVERSAL_BAR_LENGTH,
        LENGTH_TO_HANG_FROM_TRAVERSAL_BAR
    }

    public enum ClimberExtensionLimits {
        STOWED, // Corresponds to STOWED_HEIGHT
        EXTENSION_FULL_ROM,
        HIGH_BAR_TRANSFER_TO_FIXED_ARM, // Corresponds to PULLING_UP_TO_HIGH_BAR_TRANSFER_LENGTH
    }

    public enum ClimberPivotPosition {
        // If the position begins with ANGLE, rotation is CW
        // If the position ends with ANGLE, rotation is CCW

        STOWED_ANGLE,
        ANGLE_HOOK_THETA_FOR_MID_BAR,
        REACHING_FOR_HIGH_BAR_PIVOT_ANGLE,
        ANGLE_TO_HOOK_ONTO_HIGH_BAR,
        ANGLE_TO_POSITION_FIXED_ARM_FOR_HIGH_BAR_TRANSFER,
        FIXED_ARM_TO_HOOK_ONTO_HIGH_BAR_ANGLE,
        REACHING_FOR_TRAVERSAL_BAR_PIVOT_ANGLE,
        ANGLE_TO_HOOK_ONTO_TRAVERSAL_BAR
    }

    public enum ClimberPivotLimits {
        PIVOT_STOWED, // Corresponds to STOWED_ANGLE
        PIVOT_FULL_ROM,
        PIVOT_PULL_UP_TO_MID_BAR, // Corresponds to ANGLE_HOOK_THETA_FOR_MID_BAR
        PIVOT_PULL_UP_TO_HIGH_BAR, // Corresponds to ANGLE_TO_HOOK_ONTO_HIGH_BAR
        PIVOT_PULL_UP_TO_TRANSFER_HIGH_BAR, // Corresponds to ANGLE_TO_POSITION_FIXED_ARM_FOR_HIGH_BAR_TRANSFER
        PIVOT_PULL_UP_TO_TRAVERSAL_BAR, // Corresponds to ANGLE_TO_HOOK_ONTO_TRAVERSAL_BAR
    }

    public static class PeriodicIO implements Sendable {
        public boolean resetPivotLimits;
        public LimitPair pivotLimits;
        public boolean resetExtensionLimits;
        public LimitPair extensionLimits;
        public boolean releaseExtensionLowerLimit;
        public double pivotPercentOutputDemand;
        public double pivotFeedForward;
        public double pivotPositionDemandNU;
        public double extensionPercentOutputDemand;
        public double extensionPositionDemandNU;
        public boolean leftClimberLockStateDemand;
        public boolean rightClimberLockStateDemand;
        public boolean climberDiscBrakeStateDemand;
        // Inputs
        public double pivotAbsoluteEncoderPositionNU;
        public double pivotIntegratedEncoderPositionNU;
        public boolean isLeftExtensionLowerLimitClosed;
        public boolean isRightExtensionLowerLimitClosed;
        public double extensionIntegratedEncoderPosition;
        // Outputs
        private ClimberControlState pivotControlState = ClimberControlState.DISABLED;
        private ClimberControlState extensionControlState = ClimberControlState.DISABLED;
        private boolean resetPivotNeutralMode;
        private NeutralMode pivotNeutralMode;
        private boolean resetExtensionNeutralMode;
        private NeutralMode extensionNeutralMode;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("PeriodicIO");
            builder.addStringProperty("Pivot Control State", () -> pivotControlState.name(), (x) -> {
            });
            builder.addStringProperty("Extension Control State", () -> extensionControlState.name(), (x) -> {
            });
            builder.addDoubleProperty("Pivot Percent Output Demand", () -> pivotPercentOutputDemand, (x) -> {
            });
            builder.addDoubleProperty("Pivot Feed Forward", () -> pivotFeedForward, (x) -> {
            });
            builder.addDoubleProperty("Pivot Position Demand NU", () -> pivotPositionDemandNU, (x) -> {
            });
            builder.addDoubleProperty("Extension Percent Output Demand", () -> extensionPercentOutputDemand, (x) -> {
            });
            builder.addDoubleProperty("Extension Position Demand NU", () -> extensionPositionDemandNU, (x) -> {
            });
            builder.addBooleanProperty("Left Climber Lock State Demand", () -> leftClimberLockStateDemand, (x) -> {
            });
            builder.addBooleanProperty("Right Climber Lock State Demand", () -> rightClimberLockStateDemand, (x) -> {
            });
            builder.addBooleanProperty("Climber Disc Brake State Demand", () -> climberDiscBrakeStateDemand, (x) -> {
            });
            builder.addDoubleProperty("Pivot Absolute Encoder Position NU", () -> pivotAbsoluteEncoderPositionNU, (x) -> {
            });
            builder.addDoubleProperty("Pivot Integrated Encoder Position NU", () -> pivotIntegratedEncoderPositionNU, (x) -> {
            });
            builder.addBooleanProperty("Is Left Extension Lower Limit Closed", () -> isLeftExtensionLowerLimitClosed, (x) -> {
            });
            builder.addBooleanProperty("Is Right Extension Lower Limit Closed", () -> isRightExtensionLowerLimitClosed, (x) -> {
            });
            builder.addDoubleProperty("Extension Integrated Encoder Position", () -> extensionIntegratedEncoderPosition, (x) -> {
            });
        }
    }

}
