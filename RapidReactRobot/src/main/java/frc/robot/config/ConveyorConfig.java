package frc.robot.config;

public interface ConveyorConfig {

    MotorConfig getTransportControllerConfig();

    MotorConfig getFeedControllerConfig();

    double getTransportIntakePercentOutput();

    double getTransportOuttakePercentOutput();

    double getFeedOuttakePercentOutput();

    double getTransportShootPercentOutput();

    double getFeedShootPercentOutput();

}
