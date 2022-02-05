package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;

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

    }

    @Override
    public void collectData() {

    }

    @Override
    public void outputData() {

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
