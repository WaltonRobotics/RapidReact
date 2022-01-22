// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static final class ControllerPorts {

        public static final class GamepadsConfigPorts {
            public static final int kDriveGamepadPort = 0;
            public static final int kManipulationGamepadPort = 1;
        }

        public static final class JoysticksConfigPorts {
            public static final int kLeftJoystickPort = 0;
            public static final int kRightJoystickPort = 1;
            public static final int kManipulationGamepadPort = 2;
        }

    }

    public static final class DioIDs {

        public static final int kRobotID1 = 8;
        public static final int kRobotID2 = 9;

    }

    public static final class SwerveDriveConfig {
        // TODO: verify diameter and run calibration
        // 500 cm calibration = actual / odometry
        public static final double kWheelDiameterInches = 3.0;

        // From: https://github.com/strykeforce/axis-config/
        public static final double kMaxSpeedMetersPerSecond = 3.889;

        public static final double kDistanceBetweenWheelsWidthWiseMeters = Units.inchesToMeters(16.0 + 17.0 / 32.0 + 1.761652 * 2.0); // 20.055 in
        public static final double kDistanceBetweenWheelsLengthWiseMeters = Units.inchesToMeters(15.0 + 1.0 / 16.0 + 1.761652 * 2.0); // 18.586 in
        //Luke says hi

        public static final double kMaxOmega =
                (kMaxSpeedMetersPerSecond / Math.hypot(kDistanceBetweenWheelsLengthWiseMeters / 2.0,
                        kDistanceBetweenWheelsWidthWiseMeters / 2.0))
                        / 2.0; // wheel locations below

        // From: https://github.com/strykeforce/axis-config/
        static final double kDriveMotorOutputGear = 12;
        static final double kDriveInputGear = 21;
        static final double kBevelInputGear = 15;
        static final double kBevelOutputGear = 45;
        public static final int kTalonConfigTimeout = 10; // ms
        public static final double kDriveGearRatio =
                (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);


        public static Translation2d[] getWheelLocationMeters() {
            final double x = kDistanceBetweenWheelsLengthWiseMeters / 2.0; // front-back, was ROBOT_LENGTH
            final double y = kDistanceBetweenWheelsWidthWiseMeters / 2.0; // left-right, was ROBOT_WIDTH
            Translation2d[] locs = new Translation2d[4];
            locs[0] = new Translation2d(x, y); // left front
            locs[1] = new Translation2d(x, -y); // right front
            locs[2] = new Translation2d(-x, y); // left rear
            locs[3] = new Translation2d(-x, -y); // right rear
            return locs;
        }

        public static TalonFXConfiguration getDriveTalonConfig() {
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            driveConfig.supplyCurrLimit.currentLimit = 0.04;
            driveConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
            driveConfig.supplyCurrLimit.triggerThresholdTime = 40;
            driveConfig.supplyCurrLimit.enable = true;
            driveConfig.slot0.kP = 0.0051;
            driveConfig.slot0.kI = 3.01E-05;
            driveConfig.slot0.kD = 0.000;
            driveConfig.slot0.kF = 0.0455603184323331;
//            driveConfig.slot0.kP = 0.045;
//            driveConfig.slot0.kI = 0.0005;
//            driveConfig.slot0.kD = 0.000;
//            driveConfig.slot0.kF = 0.047;
            driveConfig.slot0.integralZone = 500;
            driveConfig.slot0.maxIntegralAccumulator = 75_000;
            driveConfig.slot0.allowableClosedloopError = 0;
            driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_100Ms;
            driveConfig.velocityMeasurementWindow = 64;
            driveConfig.voltageCompSaturation = 12;
            return driveConfig;
        }
    }

    public static final class SmartDashboardKeys {

        public static final String DRIVETRAIN_ROTATE_MODULES_TO_ANGLE_KEY = "Drivetrain/Rotate Modules To Angle";

        public static final String DRIVETRAIN_SETPOINT_ANGLE_DEGREES = "Drivetrain/Setpoint Angle Degrees";

        public static final String DRIVETRAIN_LEFT_FRONT_ABSOLUTE_POSITION = "Drivetrain/Left Front Absolute Counts";
        public static final String DRIVETRAIN_RIGHT_FRONT_ABSOLUTE_POSITION = "Drivetrain/Right Front Absolute Counts";
        public static final String DRIVETRAIN_LEFT_REAR_ABSOLUTE_POSITION = "Drivetrain/Left Rear Absolute Counts";
        public static final String DRIVETRAIN_RIGHT_REAR_ABSOLUTE_POSITION = "Drivetrain/Right Rear Absolute Counts";

        public static final String DRIVETRAIN_LEFT_FRONT_RELATIVE_POSITION = "Drivetrain/Left Front Relative Counts";
        public static final String DRIVETRAIN_RIGHT_FRONT_RELATIVE_POSITION = "Drivetrain/Right Front Relative Counts";
        public static final String DRIVETRAIN_LEFT_REAR_RELATIVE_POSITION = "Drivetrain/Left Rear Relative Counts";
        public static final String DRIVETRAIN_RIGHT_REAR_RELATIVE_POSITION = "Drivetrain/Right Rear Relative Counts";

        public static final String DRIVETRAIN_LEFT_FRONT_ANGLE_DEGREES = "Drivetrain/Left Front Angle Degrees";
        public static final String DRIVETRAIN_RIGHT_FRONT_ANGLE_DEGREES = "Drivetrain/Right Front Angle Degrees";
        public static final String DRIVETRAIN_LEFT_REAR_ANGLE_DEGREES = "Drivetrain/Left Rear Angle Degrees";
        public static final String DRIVETRAIN_RIGHT_REAR_ANGLE_DEGREES = "Drivetrain/Right Rear Angle Degrees";

        public static final String DRIVETRAIN_LEFT_FRONT_VELOCITY_ERROR = "Drivetrain/Left Front Velocity Error";
        public static final String DRIVETRAIN_RIGHT_FRONT_VELOCITY_ERROR = "Drivetrain/Right Front Velocity Error";
        public static final String DRIVETRAIN_LEFT_REAR_VELOCITY_ERROR = "Drivetrain/Left Rear Velocity Error";
        public static final String DRIVETRAIN_RIGHT_REAR_VELOCITY_ERROR = "Drivetrain/Right Rear Velocity Error";

        public static final String DRIVETRAIN_LEFT_FRONT_AZIMUTH_ZERO_VALUE_KEY = "Drivetrain/Left Front Azimuth Zero Value";
        public static final String DRIVETRAIN_RIGHT_FRONT_AZIMUTH_ZERO_VALUE_KEY = "Drivetrain/Right Front Azimuth Zero Value";
        public static final String DRIVETRAIN_LEFT_REAR_AZIMUTH_ZERO_VALUE_KEY = "Drivetrain/Left Rear Azimuth Zero Value";
        public static final String DRIVETRAIN_RIGHT_REAR_AZIMUTH_ZERO_VALUE_KEY = "Drivetrain/Right Rear Azimuth Zero Value";

        public static final String DRIVETRAIN_SAVE_LEFT_FRONT_AZIMUTH_ZERO_KEY = "Drivetrain/Save Left Front Azimuth Zero";
        public static final String DRIVETRAIN_SAVE_RIGHT_FRONT_AZIMUTH_ZERO_KEY = "Drivetrain/Save Right Front Azimuth Zero";
        public static final String DRIVETRAIN_SAVE_LEFT_REAR_AZIMUTH_ZERO_KEY = "Drivetrain/Save Left Rear Azimuth Zero";
        public static final String DRIVETRAIN_SAVE_RIGHT_REAR_AZIMUTH_ZERO_KEY = "Drivetrain/Save Right Rear Azimuth Zero";

    }

}
