package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.util.AccelerationLimiter;

public interface DrivetrainConfig {

    // Control-related constants
    // All arrays are in order of modules: left-front, right-front, left-rear, right-rear
    PIDController[] getAzimuthPositionalPIDs();

    TalonFXConfiguration[] getDriveControllerConfigs();

    RelativeEncoderConfig[] getAzimuthQuadratureConfigs();

    int[] getAzimuthControllerIDs();

    int[] getDriveControllerIDs();

    int[] getAbsoluteEncoderChannels();

    boolean[] getAzimuthControllerInversions();

    boolean[] getDriveControllerInversions();

    boolean[] getAbsoluteEncoderInversions();

    // Kinematics-related constants
    double getRelativeEncoderRotationsPerTick();

    double getMaxSpeedMetersPerSecond();

    double getMaxOmega();

    double getMaxFaceDirectionOmega();

    double getDriveMetersPerNU();

    Translation2d[] getWheelLocationMeters();

    AccelerationLimiter getXLimiter();

    AccelerationLimiter getYLimiter();

    AccelerationLimiter getOmegaLimiter();

    // Pathing constants/controllers
    PIDController getXController();

    PIDController getYController();

    ProfiledPIDController getThetaController();

    // Turn to angle and auto align constants

    PIDController getFaceDirectionController();

    PIDController getAutoAlignController();

    double getMinTurnOmega();

    ProfiledPIDController getTurnToAngleController();

    // Climbing-related constants
    double getClimbingMaxMetersPerSecond();

    double getClimbingMaxOmega();

}
