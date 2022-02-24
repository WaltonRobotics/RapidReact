package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public interface DrivetrainConfig {

    // Control-related constants
    // All arrays are in order of modules: left-front, right-front, left-rear, right-rear
    SmartMotionConstants[] getAzimuthControllerConfigs();

    TalonFXConfiguration[] getDriveControllerConfigs();

    int[] getAzimuthControllerIDs();

    int[] getDriveControllerIDs();

    int[] getAbsoluteEncoderChannels();

    boolean[] getAzimuthControllerInversions();

    boolean[] getDriveControllerInversions();

    boolean[] getAbsoluteEncoderInversions();

    // Kinematics-related constants
    double getRelativeEncoderRotationsPerTick();

    double getWheelDiameterInches();

    double getMaxSpeedMetersPerSecond();

    double getMaxOmega();

    double getDriveGearRatio();

    Translation2d[] getWheelLocationMeters();

    // Pathing constants/controllers
    PIDController getXController();

    PIDController getYController();

    ProfiledPIDController getThetaController();

    // Turn to angle and auto align constants

    PIDController getAutoAlignController();
    double getMinTurnOmega();

    ProfiledPIDController getTurnToAngleController();

    // Climbing-related constants
    double getClimbingMaxMetersPerSecond();
    double getClimbingMaxOmega();

}
