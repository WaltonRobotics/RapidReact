package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;

import static frc.robot.Constants.PIDProfileSlots.kShooterDefaultIndex;

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
        periodicIO.flywheelClosedLoopErrorNU = flywheelMasterController.getClosedLoopError();
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

        double currentFPGATime = Timer.getFPGATimestamp();

        if (periodicIO.lastLeftAdjustableHoodDutyCycleDemand != periodicIO.leftAdjustableHoodDutyCycleDemand) {
            periodicIO.lastAdjustableHoodChangeFPGATime = currentFPGATime;
        } else if (periodicIO.lastRightAdjustableHoodDutyCycleDemand != periodicIO.rightAdjustableHoodDutyCycleDemand) {
            periodicIO.lastAdjustableHoodChangeFPGATime = currentFPGATime;
        }

        leftAdjustableHoodServo.setSpeed(periodicIO.leftAdjustableHoodDutyCycleDemand);
        rightAdjustableHoodServo.setSpeed(periodicIO.rightAdjustableHoodDutyCycleDemand);

        periodicIO.lastLeftAdjustableHoodDutyCycleDemand = periodicIO.leftAdjustableHoodDutyCycleDemand;
        periodicIO.lastRightAdjustableHoodDutyCycleDemand = periodicIO.rightAdjustableHoodDutyCycleDemand;
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
        periodicIO.selectedProfileSlot = selectedProfileSlot;
    }

    public double getFlywheelDemand() {
        return periodicIO.flywheelDemand;
    }

    public void setFlywheelDemand(double flywheelDemand) {
        periodicIO.flywheelDemand = flywheelDemand;
    }

    public double getLeftAdjustableHoodDutyCycleDemand() {
        return periodicIO.leftAdjustableHoodDutyCycleDemand;
    }

    public void setLeftAdjustableHoodDutyCycleDemand(double leftAdjustableHoodDutyCycleDemand) {
        periodicIO.leftAdjustableHoodDutyCycleDemand = leftAdjustableHoodDutyCycleDemand;
    }

    public double getRightAdjustableHoodDutyCycleDemand() {
        return periodicIO.rightAdjustableHoodDutyCycleDemand;
    }

    public void setRightAdjustableHoodDutyCycleDemand(double rightAdjustableHoodDutyCycleDemand) {
        periodicIO.rightAdjustableHoodDutyCycleDemand = rightAdjustableHoodDutyCycleDemand;
    }

    public double getLastAdjustableHoodChangeFPGATime() {
        return periodicIO.lastAdjustableHoodChangeFPGATime;
    }

    public double getFlywheelVelocityNU() {
        return periodicIO.flywheelVelocityNU;
    }

    public double getFlywheelClosedLoopErrorNU() {
        return periodicIO.flywheelClosedLoopErrorNU;
    }

    public enum ShooterControlState {
        VELOCITY, OPEN_LOOP, DISABLED
    }

    public enum ShooterProfileSlot {
        DEFAULT_SLOT(kShooterDefaultIndex);

        ShooterProfileSlot(int idx) {
            index = idx;
        }

        private final int index;

        public int getIndex() {
            return index;
        }
    }

    public static class PeriodicIO implements Sendable {
        // Outputs
        public ShooterControlState shooterControlState;

        public ShooterProfileSlot selectedProfileSlot;
        public double flywheelDemand;
        public double leftAdjustableHoodDutyCycleDemand;
        public double rightAdjustableHoodDutyCycleDemand;
        public double lastLeftAdjustableHoodDutyCycleDemand;
        public double lastRightAdjustableHoodDutyCycleDemand;
        public double lastAdjustableHoodChangeFPGATime;
        
        // Inputs
        public double flywheelVelocityNU;
        public double flywheelClosedLoopErrorNU;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("PeriodicIO");
            builder.addStringProperty("Shooter Control State", () -> shooterControlState.name(), (x) -> {});
            builder.addStringProperty("Selected Profile Slot", () -> selectedProfileSlot.name(), (x) -> {});
            builder.addDoubleProperty("Flywheel Demand", () -> flywheelDemand, (x) -> {});
            builder.addDoubleProperty("Left Adjustable Hood Demand", () -> leftAdjustableHoodDutyCycleDemand, (x) -> {});
            builder.addDoubleProperty("Right Adjustable Hood Demand", () -> rightAdjustableHoodDutyCycleDemand, (x) -> {});
            builder.addDoubleProperty("Flywheel Velocity NU", () -> rightAdjustableHoodDutyCycleDemand, (x) -> {});
            builder.addDoubleProperty("Flywheel Closed Loop Error NU", () -> flywheelClosedLoopErrorNU, (x) -> {});
        }
    }

}
