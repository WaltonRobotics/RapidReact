package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.config.ClimberConfig;
import frc.robot.config.IntakeConfig;

import static frc.robot.RobotContainer.currentRobot;

public class Intake implements SubSubsystem {

    private final IntakeConfig config = currentRobot.getIntakeConfig();

    private final Spark leftIntakeController = new Spark(config.getLeftIntakeControllerConfig().getChannel());
    private final Spark rightIntakeController = new Spark(config.getRightIntakeControllerConfig().getChannel());

    private final Solenoid leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            config.getLeftIntakeControllerConfig().getChannel());

    private final Solenoid rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
            config.getRightIntakeControllerConfig().getChannel());

    private final PeriodicIO periodicIO = new PeriodicIO();

    @Override
    public void zeroSensors() {

    }

    @Override
    public void collectData() {

    }

    @Override
    public void outputData() {
        switch (periodicIO.intakeControlState) {
            case VOLTAGE:
                leftIntakeController.setVoltage(periodicIO.leftIntakeDemand);
                rightIntakeController.setVoltage(periodicIO.rightIntakeDemand);
                break;
            case OPEN_LOOP:
                leftIntakeController.set(periodicIO.leftIntakeDemand);
                rightIntakeController.set(periodicIO.rightIntakeDemand);
                break;
            case DISABLED:
                leftIntakeController.set(0.0);
                rightIntakeController.set(0.0);
                break;
        }

        leftIntakeSolenoid.set(periodicIO.leftIntakeDeployDemand);
        rightIntakeSolenoid.set(periodicIO.rightIntakeDeployDemand);
    }

    public Sendable getPeriodicIOSendable() {
        return periodicIO;
    }

    public IntakeControlState getIntakeControlState() {
        return periodicIO.intakeControlState;
    }

    public void setIntakeControlState(IntakeControlState intakeControlState) {
        periodicIO.intakeControlState = intakeControlState;
    }

    public double getLeftIntakeDemand() {
        return periodicIO.leftIntakeDemand;
    }

    public void setLeftIntakeDemand(double leftIntakeDemand) {
        periodicIO.leftIntakeDemand = leftIntakeDemand;
    }

    public double getRightIntakeDemand() {
        return periodicIO.rightIntakeDemand;
    }

    public void setRightIntakeDemand(double rightIntakeDemand) {
        periodicIO.rightIntakeDemand = rightIntakeDemand;
    }

    public boolean isLeftIntakeDeployStateDemand() {
        return periodicIO.leftIntakeDeployDemand;
    }

    public void setLeftIntakeDeployStateDemand(boolean leftIntakeDeployStateDemand) {
        periodicIO.leftIntakeDeployDemand = leftIntakeDeployStateDemand;
    }

    public boolean isRightIntakeDeployStateDemand() {
        return periodicIO.rightIntakeDeployDemand;
    }

    public void setRightIntakeDeployStateDemand(boolean rightIntakeDeployStateDemand) {
        periodicIO.rightIntakeDeployDemand = rightIntakeDeployStateDemand;
    }

    public enum IntakeControlState {
        VOLTAGE, OPEN_LOOP, DISABLED
    }

    public static class PeriodicIO implements Sendable {
        private IntakeControlState intakeControlState;

        // Outputs
        public double leftIntakeDemand;
        public double rightIntakeDemand;

        public boolean leftIntakeDeployDemand;
        public boolean rightIntakeDeployDemand;


        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("PeriodicIO");
            builder.addStringProperty("Intake Control State", () -> intakeControlState.name(), (x) -> {});
            builder.addDoubleProperty("Left Intake Demand", () -> leftIntakeDemand, (x) -> {});
            builder.addDoubleProperty("Right Intake Demand", () -> rightIntakeDemand, (x) -> {});
            builder.addBooleanProperty("Left Intake Deploy State Demand", () -> leftIntakeDeployDemand, (x) -> {});
            builder.addBooleanProperty("Right Intake Deploy State Demand", () -> rightIntakeDeployDemand, (x) -> {});
        }
    }

}
