package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.*;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kForward;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.kReverse;
import static frc.robot.Constants.PIDSlots.kClimberPivotPrimaryIntegratedSlot;

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

    private ClimberControlState pivotControlState;
    private ClimberControlState extensionControlState;

    @Override
    public void zeroSensors() {
        extensionController.setSelectedSensorPosition(0.0);
    }

    @Override
    public void collectData() {
        periodicIO.pivotAbsoluteEncoderPositionNU = pivotAngleAbsoluteEncoder.get();
        periodicIO.pivotIntegratedEncoderPositionNU = pivotController.getSelectedSensorPosition(kClimberPivotPrimaryIntegratedSlot);

        periodicIO.isLeftExtensionLowerLimitClosed = leftExtensionLowerLimit.get();
        periodicIO.isRightExtensionLowerLimitClosed = rightExtensionLowerLimit.get();

        periodicIO.extensionIntegratedEncoderPosition = extensionController.getSelectedSensorPosition();
    }

    @Override
    public void outputData() {
        switch (pivotControlState) {
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

        switch (extensionControlState) {
            case ZEROING:
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

    public enum ClimberControlState {
        ZEROING, AUTO, OPEN_LOOP, DISABLED
    }

    public static class PeriodicIO {
        // Outputs
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
    }

}
