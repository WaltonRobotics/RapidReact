package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public interface ClimberConfig {

    int getPivotControllerID();
    int getExtensionControllerID();
    MotorConfig getPivotControllerMotorConfig();
    MotorConfig getExtensionControllerMotorConfig();
    TalonFXConfiguration getPivotControllerTalonConfig();
    TalonFXConfiguration getExtensionControllerTalonConfig();

    EncoderConfig getPivotAngleAbsoluteEncoderConfig();

    int getLeftExtensionLowerLimitChannel();
    int getRightExtensionLowerLimitChannel();

    int getLeftClimberLockChannel();
    int getRightClimberLockChannel();

    int getClimberDiscBrakeForwardChannel();
    int getClimberDiscBrakeReverseChannel();

}
