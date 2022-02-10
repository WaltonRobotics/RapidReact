package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Conveyor implements SubSubsystem {

    private final Spark transportController = new Spark(1);
    private final Spark feedController = new Spark(2);

    private final PeriodicIO periodicIO = new PeriodicIO();
    private ConveyorControlState conveyorControlState;

    public Conveyor() {

    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void collectData() {

    }

    @Override
    public void outputData() {
        switch (conveyorControlState) {
            case VOLTAGE:
                break;
            case OPEN_LOOP:
                break;
            case DISABLED:
                break;
        }
    }

    public enum ConveyorControlState {
        VOLTAGE, OPEN_LOOP, DISABLED
    }

    public static class PeriodicIO {
        // Outputs
        public double transportDemand;
        public double feedDemand;
    }

}
