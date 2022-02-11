package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.*;
import frc.robot.util.EnhancedBoolean;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static frc.robot.Constants.Climber.kExtensionZeroingPercentOutput;

public class Climber implements SubSubsystem {

    private final Encoder pivotAngleAbsoluteEncoder = new Encoder(0, 1);

    private final DigitalInput leftExtensionLowerLimit = new DigitalInput(0);
    private final DigitalInput rightExtensionLowerLimit = new DigitalInput(1);

    private final TalonFX pivotController = new TalonFX(0);
    private final TalonFX extensionController = new TalonFX(1);

    private final Solenoid leftClimberLock = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid rightClimberLock = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private final DoubleSolenoid climberDiscBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    private final PeriodicIO periodicIO = new PeriodicIO();

    private EnhancedBoolean isZeroed = new EnhancedBoolean();

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
