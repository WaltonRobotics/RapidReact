package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public class DrivetrainConfig {

    public SmartMotionConstants azimuthControllerConfig;
    public TalonFXConfiguration driveControllerConfig;

    public boolean[] azimuthControllerInversions;
    public boolean[] driveControllerInversions;
    public boolean[] absoluteEncoderInversions;

}
