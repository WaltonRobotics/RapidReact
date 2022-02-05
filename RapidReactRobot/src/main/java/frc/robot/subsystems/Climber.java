package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

import static frc.robot.Constants.PIDSlots.kClimberPivotAuxiliaryAbsoluteSlot;
import static frc.robot.Constants.PIDSlots.kClimberPivotPrimaryIntegratedSlot;

public class Climber implements SubSubsystem {

    private final DigitalInput leftExtensionLowerLimit = new DigitalInput(0);
    private final DigitalInput rightExtensionLowerLimit = new DigitalInput(1);

    private final TalonFX pivotController = new TalonFX(0);
    private final TalonFX extensionController = new TalonFX(1);

    private final Solenoid leftClimberLock = new Solenoid(PneumaticsModuleType.CTREPCM, 0);
    private final Solenoid rightClimberLock = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private final DoubleSolenoid climberDiscBrake = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, 2, 3);

    private final PeriodicIO periodicIO = new PeriodicIO();
    private ClimberControlState climberControlState;

    @Override
    public void zeroSensors() {
        extensionController.setSelectedSensorPosition(0.0);
    }

    @Override
    public void collectData() {
        periodicIO.isLeftExtensionLowerLimitClosed = leftExtensionLowerLimit.get();
        periodicIO.isRightExtensionLowerLimitClosed = rightExtensionLowerLimit.get();

        periodicIO.pivotAbsoluteEncoderPosition = pivotController.getSelectedSensorPosition(kClimberPivotAuxiliaryAbsoluteSlot);
        periodicIO.pivotIntegratedEncoderPosition = pivotController.getSelectedSensorPosition(kClimberPivotPrimaryIntegratedSlot);

        periodicIO.extensionIntegratedEncoderPosition = extensionController.getSelectedSensorPosition();
    }

    @Override
    public void outputData() {
        switch (climberControlState) {
            case ZEROING:
                break;
            case AUTO:
                break;
            case OPEN_LOOP:
                break;
            case DISABLED:
                break;
        }

        leftClimberLock.set(periodicIO.leftClimberLockStateDemand);
        rightClimberLock.set(periodicIO.rightClimberLockStateDemand);
        climberDiscBrake.set(periodicIO.climberDiscBrakeStateDemand ? DoubleSolenoid.Value.kForward : DoubleSolenoid.Value.kReverse);
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
        public boolean isLeftExtensionLowerLimitClosed;
        public boolean isRightExtensionLowerLimitClosed;

        public double pivotAbsoluteEncoderPosition;
        public double pivotIntegratedEncoderPosition;

        public double extensionIntegratedEncoderPosition;
    }

}
