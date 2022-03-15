package frc.robot.config;

public interface IntakeConfig {

    MotorConfig getLeftIntakeControllerConfig();

    MotorConfig getRightIntakeControllerConfig();

    int getLeftIntakeSolenoidChannel();
    int getRightIntakeSolenoidChannel();

    double getLeftIntakePercentOutput();
    double getRightIntakePercentOutput();

    double getLeftOuttakePercentOutput();
    double getRightOuttakePercentOutput();

}
