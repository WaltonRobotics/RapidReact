package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.ShooterConfig;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.PIDProfileSlots.kShooterDefaultIndex;
import static frc.robot.Constants.PIDProfileSlots.kSpinUpIndex;
import static frc.robot.Constants.PIDProfileSlots.kShooterIndex;
import static frc.robot.RobotContainer.currentRobot;

public class Shooter implements SubSubsystem {

    private final ShooterConfig config = currentRobot.getShooterConfig();
    private HoodPosition hoodPosition;

    private final TalonFX flywheelMasterController = new TalonFX(
            config.getFlywheelMasterControllerMotorConfig().getChannelOrID());
    private final TalonFX flywheelSlaveController = new TalonFX(
            config.getFlywheelSlaveControllerMotorConfig().getChannelOrID());

    private final Servo leftAdjustableHoodServo = new Servo(
            config.getLeftAdjustableHoodServoConfig().getChannelOrID());
    private final Servo rightAdjustableHoodServo = new Servo(
            config.getRightAdjustableHoodServoConfig().getChannelOrID());

    private final PeriodicIO periodicIO = new PeriodicIO();

    public Shooter() {
        flywheelMasterController.configFactoryDefault(10);
        flywheelMasterController.configAllSettings(config.getFlywheelMasterControllerTalonConfig(), 10);
        flywheelMasterController.setInverted(config.getFlywheelMasterControllerMotorConfig().isInverted());
        flywheelMasterController.setSensorPhase(config.getFlywheelMasterControllerMotorConfig().isInverted());
        flywheelMasterController.setNeutralMode(NeutralMode.Coast);
        flywheelMasterController.enableVoltageCompensation(true);

        flywheelSlaveController.configFactoryDefault(10);
        flywheelSlaveController.configAllSettings(config.getFlywheelSlaveControllerTalonConfig(), 10);
        flywheelSlaveController.setInverted(config.getFlywheelSlaveControllerMotorConfig().isInverted());
        flywheelSlaveController.setSensorPhase(config.getFlywheelSlaveControllerMotorConfig().isInverted());
        flywheelSlaveController.setNeutralMode(NeutralMode.Coast);
        flywheelMasterController.enableVoltageCompensation(false);

        // From L16-R datasheet
        leftAdjustableHoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
        rightAdjustableHoodServo.setBounds(2.0, 1.8, 1.5, 1.2, 1.0);
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void collectData() {
        periodicIO.flywheelVelocityNU = flywheelMasterController.getSelectedSensorVelocity();
        periodicIO.flywheelClosedLoopErrorNU = flywheelMasterController.getClosedLoopError();
        LimelightHelper.updateData();
    }

    @Override
    public void outputData() {
        int masterID = config.getFlywheelMasterControllerMotorConfig().getChannelOrID();

        switch (periodicIO.shooterControlState) {
            case VELOCITY:
                flywheelMasterController.set(ControlMode.Velocity, periodicIO.flywheelDemand);
                flywheelSlaveController.set(TalonFXControlMode.Follower, masterID);
                break;
            case OPEN_LOOP:
                flywheelMasterController.set(ControlMode.PercentOutput, periodicIO.flywheelDemand);
                flywheelSlaveController.set(TalonFXControlMode.Follower, masterID);
                break;
            case DISABLED:
                flywheelMasterController.set(ControlMode.Disabled, 0.0);
                flywheelSlaveController.set(TalonFXControlMode.Follower, masterID);
                break;
        }

        double currentFPGATime = Timer.getFPGATimestamp();

        if (periodicIO.lastLeftAdjustableHoodDutyCycleDemand != periodicIO.leftAdjustableHoodDutyCycleDemand) {
            periodicIO.lastAdjustableHoodChangeFPGATime = currentFPGATime;
        }

        if (periodicIO.lastRightAdjustableHoodDutyCycleDemand != periodicIO.rightAdjustableHoodDutyCycleDemand) {
            periodicIO.lastAdjustableHoodChangeFPGATime = currentFPGATime;
        }

        leftAdjustableHoodServo.setSpeed(periodicIO.leftAdjustableHoodDutyCycleDemand);
        rightAdjustableHoodServo.setSpeed(periodicIO.rightAdjustableHoodDutyCycleDemand);

        periodicIO.lastLeftAdjustableHoodDutyCycleDemand = periodicIO.leftAdjustableHoodDutyCycleDemand;
        periodicIO.lastRightAdjustableHoodDutyCycleDemand = periodicIO.rightAdjustableHoodDutyCycleDemand;
    }

    @Override
    public Sendable getPeriodicIOSendable() {
        return periodicIO;
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

    public void setHoodPosition(HoodPosition selectedHoodPosition){
        hoodPosition = selectedHoodPosition;
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

    public double getCurrentFPGATime(){
        return Timer.getFPGATimestamp();
    }

    public double getFlywheelVelocityNU() {
        return periodicIO.flywheelVelocityNU;
    }

    public double getFlywheelClosedLoopErrorNU() {
        return periodicIO.flywheelClosedLoopErrorNU;
    }

    public HoodPosition getHoodPosition(){
        return hoodPosition;
    }

    public ShooterConfig getConfig() {
        return config;
    }

    //hood 1 is 6o degrees
    public double getHoodOneEstimatedVelocityFromTarget(){

        //may want to use minimum from absolute shooting distances
        double distanceFeet = LimelightHelper.getDistanceToTargetFeet();
        InterpolatingDouble result;
        result = getConfig().getShooterMap().getInterpolated(new InterpolatingDouble(distanceFeet));
        return result.value;
//        if(result != null ){
//            return result.value;
//        }
//        else{
//            return kShooterMap.getInterpolated(new InterpolatingDouble(kDefaultShootingDistanceFeet)).value;
//        }
    }

    //hood 2 is 70 degrees
    public double getHoodTwoEstimatedVelocityFromTarget(){
        double distanceFeet = LimelightHelper.getDistanceToTargetFeet();
        InterpolatingDouble result;
        result = getConfig().getShooterMap2().getInterpolated(new InterpolatingDouble(distanceFeet));
        return result.value;
    }

    public enum ShooterControlState {
        VELOCITY, OPEN_LOOP, DISABLED
    }

    public enum ShooterProfileSlot {
        DEFAULT_SLOT(kShooterDefaultIndex),
        SHOOT_SLOT(kShooterIndex),
        SPIN_UP_SLOT(kSpinUpIndex);

        private final int index;

        ShooterProfileSlot(int idx) {
            index = idx;
        }

        public int getIndex() {
            return index;
        }
    }

    public enum HoodPosition {
        SIXTY_DEGREES,
        SEVENTY_DEGREES;
    }

    public static class PeriodicIO implements Sendable {
        // Outputs
        public ShooterControlState shooterControlState = ShooterControlState.DISABLED;

        public ShooterProfileSlot selectedProfileSlot = ShooterProfileSlot.DEFAULT_SLOT;
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
            builder.addStringProperty("Shooter Control State", () -> shooterControlState.name(), (x) -> {
            });
            builder.addStringProperty("Selected Profile Slot", () -> selectedProfileSlot.name(), (x) -> {
            });
            builder.addDoubleProperty("Flywheel Demand", () -> flywheelDemand, (x) -> {
            });
            builder.addDoubleProperty("Left Adjustable Hood Demand", () -> leftAdjustableHoodDutyCycleDemand, (x) -> {
            });
            builder.addDoubleProperty("Right Adjustable Hood Demand", () -> rightAdjustableHoodDutyCycleDemand, (x) -> {
            });
            builder.addDoubleProperty("Flywheel Velocity NU", () -> rightAdjustableHoodDutyCycleDemand, (x) -> {
            });
            builder.addDoubleProperty("Flywheel Closed Loop Error NU", () -> flywheelClosedLoopErrorNU, (x) -> {
            });
        }
    }

}
