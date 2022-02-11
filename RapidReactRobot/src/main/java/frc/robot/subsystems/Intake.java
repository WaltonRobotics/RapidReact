package frc.robot.subsystems;

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

    public static class PeriodicIO {
        private IntakeControlState intakeControlState;

        // Outputs
        public double leftIntakeDemand;
        public double rightIntakeDemand;

        public boolean leftIntakeDeployStateDemand;
        public boolean rightIntakeDeployStateDemand;
    }

}
