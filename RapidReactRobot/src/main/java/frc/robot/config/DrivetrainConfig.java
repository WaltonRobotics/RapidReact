package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;

public class DrivetrainConfig {

    // Control-related constants
    // All arrays are in order of modules: left-front, right-front, left-rear, right-rear
    public SmartMotionConstants[] azimuthControllerConfigs = new SmartMotionConstants[4];
    public TalonFXConfiguration[] driveControllerConfigs = new TalonFXConfiguration[4];

    public int[] azimuthControllerIDs;
    public int[] driveControllerIDs;
    public int[] absoluteEncoderChannels;

    public boolean[] azimuthControllerInversions;
    public boolean[] driveControllerInversions;
    public boolean[] absoluteEncoderInversions;

    // Kinematics-related constants
    public double kRelativeEncoderRotationsPerTick;
    public double kWheelDiameterInches;

    public double kMaxSpeedMetersPerSecond;
    public double kMaxOmega;

    public int kTalonConfigTimeout;
    public double kDriveGearRatio;

    public Translation2d[] wheelLocationMeters;

    // Pathing constants/controllers
    public PIDController xController;
    public PIDController yController;
    public ProfiledPIDController thetaController;

}
