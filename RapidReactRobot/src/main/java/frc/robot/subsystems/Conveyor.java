package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.config.ConveyorConfig;

import static frc.robot.RobotContainer.currentRobot;

public class Conveyor implements SubSubsystem {

    private final ConveyorConfig config = currentRobot.getConveyorConfig();

    private final VictorSPX transportController = new VictorSPX(config.getTransportControllerConfig().getChannelOrID());
    private final VictorSPX feedController = new VictorSPX(config.getFeedControllerConfig().getChannelOrID());

    private final PeriodicIO periodicIO = new PeriodicIO();

    public Conveyor() {
        transportController.setInverted(config.getTransportControllerConfig().isInverted());
        feedController.setInverted(config.getFeedControllerConfig().isInverted());

        transportController.configVoltageCompSaturation(12.0);
        transportController.enableVoltageCompensation(true);

        feedController.configVoltageCompSaturation(12.0);
        feedController.enableVoltageCompensation(true);
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
            case OPEN_LOOP:
                transportController.set(VictorSPXControlMode.PercentOutput, periodicIO.transportDemand);
                feedController.set(VictorSPXControlMode.PercentOutput, periodicIO.feedDemand);
                break;
            case DISABLED:
                transportController.set(VictorSPXControlMode.PercentOutput, 0.0);
                feedController.set(VictorSPXControlMode.PercentOutput, 0.0);
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
        OPEN_LOOP, DISABLED
    }

    public ConveyorConfig getConfig() {
        return config;
    }

    public static class PeriodicIO implements Sendable {
        // Outputs
        public ConveyorControlState conveyorControlState = ConveyorControlState.DISABLED;

        public double transportDemand;
        public double feedDemand;

        @Override
        public void initSendable(SendableBuilder builder) {
            builder.setSmartDashboardType("PeriodicIO");
            builder.addStringProperty("Conveyor Control State", () -> conveyorControlState.name(), (x) -> {
            });
            builder.addDoubleProperty("Transport Demand", () -> transportDemand, (x) -> {
            });
            builder.addDoubleProperty("Feed Demand", () -> feedDemand, (x) -> {
            });
        }
    }

}
