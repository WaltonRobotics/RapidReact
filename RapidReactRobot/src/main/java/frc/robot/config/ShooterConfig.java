package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import frc.robot.subsystems.Shooter;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.util.interpolation.InterpolatingTreeMap;

import java.util.HashMap;

public interface ShooterConfig {

    MotorConfig getFlywheelMasterControllerMotorConfig();

    MotorConfig getFlywheelSlaveControllerMotorConfig();

    TalonFXConfiguration getFlywheelMasterControllerTalonConfig();

    TalonFXConfiguration getFlywheelSlaveControllerTalonConfig();

    MotorConfig getAdjustableHoodServoConfig();

    double getLimelightMountingHeightInches();
    double getLimelightMountingAngleDegrees();

    HashMap<Shooter.HoodPosition, Target> getHoodTargets();
    HashMap<Shooter.HoodPosition, InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble>> getHoodMaps();

}
