package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrameEnhanced;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.*;
import frc.robot.config.ClimberConfig;
import frc.robot.util.EnhancedBoolean;

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

    private EnhancedBoolean isZeroed = new EnhancedBoolean();

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
        pivotController.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100);

        extensionController.configFactoryDefault(10);
        extensionController.configAllSettings(config.getExtensionControllerTalonConfig(), 10);
        extensionController.setInverted(config.getExtensionControllerMotorConfig().isInverted());
        extensionController.setSensorPhase(config.getExtensionControllerMotorConfig().isInverted());
        extensionController.setNeutralMode(NeutralMode.Brake);
        extensionController.enableVoltageCompensation(true);
        extensionController.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 100);
    }

    @Override
    public void zeroSensors() {
        extensionController.setSelectedSensorPosition(0.0);
    }

    @Override
    public void collectData() {
        periodicIO.pivotAbsoluteEncoderPositionNU = pivotAngleAbsoluteEncoder.get();
        periodicIO.pivotIntegratedEncoderPositionNU = pivotController.getSelectedSensorPosition();

        periodicIO.isLeftExtensionLowerLimitClosed = leftExtensionLowerLimit.get();
        periodicIO.isRightExtensionLowerLimitClosed = rightExtensionLowerLimit.get();

        periodicIO.extensionIntegratedEncoderPosition = extensionController.getSelectedSensorPosition();
    }

    @Override
    public void outputData() {
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

    public Sendable getPeriodicIOSendable() {
        return periodicIO;
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

    public enum ClimberControlState {
        ZEROING, AUTO, OPEN_LOOP, DISABLED
    }

    public static class PeriodicIO implements Sendable {
        // Outputs
        private ClimberControlState pivotControlState;
        private ClimberControlState extensionControlState;

        public double pivotPercentOutputDemand;
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

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("PeriodicIO");
            builder.addStringProperty("Pivot Control State", () -> pivotControlState.name(), (x) -> {});
            builder.addStringProperty("Extension Control State", () -> extensionControlState.name(), (x) -> {});
            builder.addDoubleProperty("Pivot Percent Output Demand", () -> pivotPercentOutputDemand, (x) -> {});
            builder.addDoubleProperty("Pivot Position Demand NU", () -> pivotPercentOutputDemand, (x) -> {});
            builder.addDoubleProperty("Extension Percent Output Demand", () -> pivotPercentOutputDemand, (x) -> {});
            builder.addDoubleProperty("Extension Position Demand NU", () -> pivotPercentOutputDemand, (x) -> {});
            builder.addBooleanProperty("Left Climber Lock State Demand", () -> leftClimberLockStateDemand, (x) -> {});
            builder.addBooleanProperty("Right Climber Lock State Demand", () -> rightClimberLockStateDemand, (x) -> {});
            builder.addBooleanProperty("Climber Disc Brake State Demand", () -> climberDiscBrakeStateDemand, (x) -> {});
            builder.addDoubleProperty("Pivot Absolute Encoder Position NU", () -> pivotAbsoluteEncoderPositionNU, (x) -> {});
            builder.addDoubleProperty("Pivot Integrated Encoder Position NU", () -> pivotIntegratedEncoderPositionNU, (x) -> {});
            builder.addBooleanProperty("Is Left Extension Lower Limit Closed", () -> isLeftExtensionLowerLimitClosed, (x) -> {});
            builder.addBooleanProperty("Is Right Extension Lower Limit Closed", () -> isRightExtensionLowerLimitClosed, (x) -> {});
            builder.addDoubleProperty("Extension Integrated Encoder Position", () -> extensionIntegratedEncoderPosition, (x) -> {});
        }
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
        periodicIO.pivotPositionDemandNU = pivotPositionDemandNU;
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

    public boolean getLeftClimberLockStateDemand() {
        return periodicIO.leftClimberLockStateDemand;
    }

    public void setLeftClimberLockStateDemand(boolean leftClimberLockStateDemand) {
        periodicIO.leftClimberLockStateDemand = leftClimberLockStateDemand;
    }

    public boolean getRightClimberLockStateDemand() {
        return periodicIO.rightClimberLockStateDemand;
    }

    public void setRightClimberLockStateDemand(boolean rightClimberLockStateDemand) {
        periodicIO.rightClimberLockStateDemand = rightClimberLockStateDemand;
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

}
