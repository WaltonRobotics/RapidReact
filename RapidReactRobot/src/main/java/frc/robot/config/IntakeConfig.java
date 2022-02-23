package frc.robot.config;

public interface IntakeConfig {

    MotorConfig getLeftIntakeControllerConfig();

    MotorConfig getRightIntakeControllerConfig();

    double getLeftIntakeVoltage();
    double getRightIntakeVoltage();

}
