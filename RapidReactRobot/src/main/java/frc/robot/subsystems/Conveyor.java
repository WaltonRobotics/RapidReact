package frc.robot.subsystems;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.config.ConveyorConfig;

import static frc.robot.RobotContainer.currentRobot;

public class Conveyor implements SubSubsystem {

    private final ConveyorConfig config = currentRobot.getConveyorConfig();

    private final Spark transportController = new Spark(config.getTransportControllerConfig().getChannelOrID());
    private final Spark feedController = new Spark(config.getFeedControllerConfig().getChannelOrID());

    private final PeriodicIO periodicIO = new PeriodicIO();

    public Conveyor() {
        transportController.setInverted(config.getTransportControllerConfig().isInverted());
        feedController.setInverted(config.getFeedControllerConfig().isInverted());
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

    @Override
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
