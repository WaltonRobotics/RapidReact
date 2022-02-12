package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
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

    }

    public Sendable getPeriodicIOSendable() {
        return periodicIO;
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

    public static class PeriodicIO implements Sendable {
        // Outputs
        public ConveyorControlState conveyorControlState;

        public double transportDemand;
        public double feedDemand;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("PeriodicIO");
            builder.addStringProperty("Conveyor Control State", () -> conveyorControlState.name(), (x) -> {});
            builder.addDoubleProperty("Transport Demand", () -> transportDemand, (x) -> {});
            builder.addDoubleProperty("Feed Demand", () -> feedDemand, (x) -> {});
        }
    }

}
