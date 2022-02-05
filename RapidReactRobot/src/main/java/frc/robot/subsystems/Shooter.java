package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.wpilibj.Servo;


public class Shooter implements SubSubsystem {

    private final TalonFX flywheelMasterController = new TalonFX(1);
    private final TalonFX flywheelSlaveController = new TalonFX(2);

    private final Servo leftAdjustableHoodServo = new Servo(0);
    private final Servo rightAdjustableHoodServo = new Servo(1);

    private final PeriodicIO periodicIO = new PeriodicIO();
    private ShooterControlState shooterControlState;

    @Override
    public void zeroSensors() {

    }

    @Override
    public void collectData() {

    }

    @Override
    public void outputData() {

    }

    public enum ShooterControlState {
        VELOCITY, OPEN_LOOP, DISABLED
    }

    public static class PeriodicIO {
        // Outputs
        public double flywheelDemand;
        public double leftAdjustableHoodDutyCycleDemand;
        public double rightAdjustableHoodDutyCycleDemand;

        // Inputs
        public double flywheelVelocityNU;
    }

}
