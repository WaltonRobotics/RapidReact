package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;

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

    int getTalonConfigTimeout();
    double getDriveGearRatio();

    Translation2d[] getWheelLocationMeters();

    // Pathing constants/controllers
    PIDController getXController();
    PIDController getYController();
    ProfiledPIDController getThetaController();

}
