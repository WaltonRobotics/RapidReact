package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Intake implements SubSubsystem {

    private final Spark leftIntakeController = new Spark(1);
    private final Spark rightIntakeController = new Spark(2);

    private final Solenoid leftIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 1);
    private final Solenoid rightIntakeSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM, 2);

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

        leftIntakeSolenoid.set(periodicIO.leftIntakeDeployStateDemand);
        rightIntakeSolenoid.set(periodicIO.rightIntakeDeployStateDemand);
    }

    public enum IntakeControlState {
        VOLTAGE, OPEN_LOOP, DISABLED
    }

    public static class PeriodicIO implements Sendable {
        private IntakeControlState intakeControlState;

        // Outputs
        public double leftIntakeDemand;
        public double rightIntakeDemand;

        public boolean leftIntakeDeployStateDemand;
        public boolean rightIntakeDeployStateDemand;


        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("PeriodicIO");
            builder.addStringProperty("Intake Control State", () -> intakeControlState.name(), (x) -> {});
            builder.addDoubleProperty("Left Intake Demand", () -> leftIntakeDemand, (x) -> {});
            builder.addDoubleProperty("Right Intake Demand", () -> rightIntakeDemand, (x) -> {});
            builder.addBooleanProperty("Left Intake Deploy State", () -> leftIntakeDeployStateDemand, (x) -> {});
            builder.addBooleanProperty("Right Intake Deploy State", () -> rightIntakeDeployStateDemand, (x) -> {});
        }
    }

}
