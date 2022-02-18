package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public interface ShooterConfig {

    MotorConfig getFlywheelMasterControllerMotorConfig();

    MotorConfig getFlywheelSlaveControllerMotorConfig();

    TalonFXConfiguration getFlywheelMasterControllerTalonConfig();

    TalonFXConfiguration getFlywheelSlaveControllerTalonConfig();

    MotorConfig getLeftAdjustableHoodServoConfig();

    MotorConfig getRightAdjustableHoodServoConfig();

    double getMountingHeightInches();

    double getMountingAngleDegrees();
}
