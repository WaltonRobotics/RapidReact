package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.VictorSPXControlMode;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.config.ConveyorConfig;

import static frc.robot.Constants.ContextFlags.kIsInCompetition;
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

        transportController.setNeutralMode(NeutralMode.Brake);
        feedController.setNeutralMode(NeutralMode.Brake);

        configTransportStatusFrame();
        configFeedStatusFrame();
    }

    @Override
    public void zeroSensors() {

    }

    @Override
    public void collectData() {
        periodicIO.hasTransportControllerResetOccurred = transportController.hasResetOccurred();
        periodicIO.hasFeedControllerResetOccurred = feedController.hasResetOccurred();
    }

    @Override
    public void outputData() {
        if (periodicIO.hasTransportControllerResetOccurred) {
            configTransportStatusFrame();
        }

        if (periodicIO.hasFeedControllerResetOccurred) {
            configFeedStatusFrame();
        }

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
    public void updateShuffleboard() {
//        SmartDashboard.putString("Conveyor Control State", periodicIO.conveyorControlState.name());
//        SmartDashboard.putNumber("Transport Demand", periodicIO.transportDemand);
//        SmartDashboard.putNumber("Feed Demand", periodicIO.feedDemand);
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

    private void configTransportStatusFrame() {
        transportController.setStatusFramePeriod(StatusFrame.Status_1_General, 200);
        transportController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
        transportController.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 200);
        transportController.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 1000);
        transportController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 1000);
        transportController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
        transportController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 1000);
        transportController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
        transportController.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);
        transportController.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000);
    }

    private void configFeedStatusFrame() {
        feedController.setStatusFramePeriod(StatusFrame.Status_1_General, 200);
        feedController.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 1000);
        feedController.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 200);
        feedController.setStatusFramePeriod(StatusFrame.Status_9_MotProfBuffer, 1000);
        feedController.setStatusFramePeriod(StatusFrame.Status_10_Targets, 1000);
        feedController.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
        feedController.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 1000);
        feedController.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
        feedController.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);
        feedController.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000);
    }

    public enum ConveyorControlState {
        OPEN_LOOP, DISABLED
    }

    public ConveyorConfig getConfig() {
        return config;
    }

    public static class PeriodicIO {
        // Inputs
        public boolean hasTransportControllerResetOccurred;
        public boolean hasFeedControllerResetOccurred;

        // Outputs
        public ConveyorControlState conveyorControlState = ConveyorControlState.DISABLED;

        public double transportDemand;
        public double feedDemand;
    }

}
