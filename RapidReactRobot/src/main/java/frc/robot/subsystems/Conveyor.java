package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Conveyor implements SubSubsystem {

    private final Spark transportController = new Spark(1);
    private final Spark feedController = new Spark(2);

    private final PeriodicIO periodicIO = new PeriodicIO();

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
        switch (periodicIO.conveyorControlState) {
            case VOLTAGE:
                transportController.setVoltage(periodicIO.transportDemand);
                feedController.setVoltage(periodicIO.feedDemand);
                break;
            case OPEN_LOOP:
                transportController.set(periodicIO.transportDemand);
                feedController.set(periodicIO.feedDemand);
                break;
            case DISABLED:
                transportController.set(0.0);
                feedController.set(0.0);
                break;
        }
    }

    public ConveyorControlState getConveyorControlState() {
        return periodicIO.conveyorControlState;
    }

    public void setConveyorControlState(ConveyorControlState conveyorControlState) {
        periodicIO.conveyorControlState = conveyorControlState;
    }

    public double getTransportDemand() {
        return periodicIO.transportDemand;
    }

    public void setTransportDemand(double transportDemand) {
        periodicIO.transportDemand = transportDemand;
    }

    public double getFeedDemand() {
        return periodicIO.feedDemand;
    }

    public void setFeedDemand(double feedDemand) {
        periodicIO.feedDemand = feedDemand;
    }

    public enum ConveyorControlState {
        VOLTAGE, OPEN_LOOP, DISABLED
    }

    public static class PeriodicIO {
        // Outputs
        public ConveyorControlState conveyorControlState;

        public double transportDemand;
        public double feedDemand;
    }

}
