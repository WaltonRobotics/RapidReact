package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Servo;


public class Shooter implements SubSubsystem {

    private final int kFlywheelMasterID = 1;

    private final TalonFX flywheelMasterController = new TalonFX(kFlywheelMasterID);
    private final TalonFX flywheelSlaveController = new TalonFX(2);

    private final Servo leftAdjustableHoodServo = new Servo(0);
    private final Servo rightAdjustableHoodServo = new Servo(1);

    private final PeriodicIO periodicIO = new PeriodicIO();

    @Override
    public void zeroSensors() {

    }

    @Override
    public void collectData() {
        periodicIO.flywheelVelocityNU = flywheelMasterController.getSelectedSensorVelocity();
        periodicIO.flywheelClosedLoopErrorNU = flywheelMasterController.getClosedLoopError(periodicIO.selectedPIDSlot);
    }

    @Override
    public void outputData() {
        switch (periodicIO.shooterControlState) {
            case VELOCITY:
                flywheelMasterController.set(ControlMode.Velocity, periodicIO.flywheelDemand);
                flywheelSlaveController.set(TalonFXControlMode.Follower, kFlywheelMasterID);
                break;
            case OPEN_LOOP:
                flywheelMasterController.set(ControlMode.PercentOutput, periodicIO.flywheelDemand);
                flywheelSlaveController.set(TalonFXControlMode.Follower, kFlywheelMasterID);
                break;
            case DISABLED:
                flywheelMasterController.set(ControlMode.Disabled, 0.0);
                flywheelSlaveController.set(TalonFXControlMode.Follower, kFlywheelMasterID);
                break;
        }

        leftAdjustableHoodServo.setSpeed(periodicIO.leftAdjustableHoodDutyCycleDemand);
        rightAdjustableHoodServo.setSpeed(periodicIO.rightAdjustableHoodDutyCycleDemand);
    }

    public enum ShooterControlState {
        VELOCITY, OPEN_LOOP, DISABLED
    }

    public static class PeriodicIO {
        // Outputs
        public ShooterControlState shooterControlState;

        public int selectedPIDSlot;
        public double flywheelDemand;
        public double leftAdjustableHoodDutyCycleDemand;
        public double rightAdjustableHoodDutyCycleDemand;

        // Inputs
        public double flywheelVelocityNU;
        public double flywheelClosedLoopErrorNU;
    }

}
