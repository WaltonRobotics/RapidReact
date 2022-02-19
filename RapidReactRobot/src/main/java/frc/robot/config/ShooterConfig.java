package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.util.interpolation.InterpolatingTreeMap;

public interface ShooterConfig {

    MotorConfig getFlywheelMasterControllerMotorConfig();

    MotorConfig getFlywheelSlaveControllerMotorConfig();

    TalonFXConfiguration getFlywheelMasterControllerTalonConfig();

    TalonFXConfiguration getFlywheelSlaveControllerTalonConfig();

    MotorConfig getLeftAdjustableHoodServoConfig();

    MotorConfig getRightAdjustableHoodServoConfig();

    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterMap();
    InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterMap2();

    double getMountingHeightInches();

    double getMountingAngleDegrees();
}
