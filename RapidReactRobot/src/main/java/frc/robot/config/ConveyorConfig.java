package frc.robot.config;

public interface ConveyorConfig {

    MotorConfig getTransportControllerConfig();

    MotorConfig getFeedControllerConfig();

    double getTransportIntakeVoltage();

    double getTransportOuttakeVoltage();
    double getFeedOuttakeVoltage();

    double getTransportShootVoltage();
    double getFeedShootVoltage();

}
