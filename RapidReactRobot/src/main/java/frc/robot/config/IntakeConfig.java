package frc.robot.config;

public interface IntakeConfig {

    MotorConfig getLeftIntakeControllerConfig();

    MotorConfig getRightIntakeControllerConfig();

    int getLeftSolenoidChannel();
    int getRightSolenoidChannel();

    double getLeftIntakePercentOutput();
    double getRightIntakePercentOutput();

    double getLeftOuttakePercentOutput();
    double getRightOuttakePercentOutput();

}
