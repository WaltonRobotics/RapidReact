// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
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

    public static final class ContextFlags {

        public static final boolean kIsInCompetition = false;
        public static final boolean kIsInTuningMode = true;

    }

    public static final class ControllerPorts {

        public static final class XboxConfigPorts {
            public static final int kDriveXboxControllerPort = 0;
            public static final int kManipulationXboxControllerPort = 1;
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

        public static final double kDistanceBetweenWheelsWidthWiseMeters = Units.inchesToMeters(15.0 + 1.0 / 16.0 + 1.761652 * 2.0); // 18.586 in
        public static final double kDistanceBetweenWheelsLengthWiseMeters = Units.inchesToMeters(16.0 + 17.0 / 32.0 + 1.761652 * 2.0); // 20.055 in

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

        // Pathing constants/controllers
        static final double kTranslationalP = 6.0;
        static final double kTranslationalD = kTranslationalP / 100.0;
        public static final PIDController kXController = new PIDController(kTranslationalP, 0.0, kTranslationalD);
        public static final PIDController kYController = new PIDController(kTranslationalP, 0.0, kTranslationalD);
        public static final ProfiledPIDController kThetaController =
                new ProfiledPIDController(
                        3.5,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(kMaxOmega / 2.0, 3.14));

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

    public static class VisionConstants {

        public static final int kLEDsOff = 1;
        public static final int kLEDsOn = 3;

        public static final int kVisionMode = 0;
        public static final int kDriverMode = 1;

        public static final int kTxWindowSize = 1;
        public static final int kTyWindowSize = 5;
        public static final int kCamtranWindowSize = 5;

        public static final int kAlignmentPipeline = 0;

        public static final double kAlignmentTimeout = 1.5;

    }

    public static final class SmartDashboardKeys {

        public static final String kDrivetrainSetModuleStatesKey = "Drivetrain/Set Module States";
        public static final String kDrivetrainSetpointAngleDegreesKey = "Drivetrain/Setpoint Angle Degrees";
        public static final String kDrivetrainSetpointVelocityKey = "Drivetrain/Setpoint Velocity Msec";

        public static final String kDrivetrainLeftFrontAbsolutePositionKey = "Drivetrain/Left Front Absolute Counts";
        public static final String kDrivetrainRightFrontAbsolutePositionKey = "Drivetrain/Right Front Absolute Counts";
        public static final String kDrivetrainLeftRearAbsolutePositionKey = "Drivetrain/Left Rear Absolute Counts";
        public static final String kDrivetrainRightRearAbsolutePositionKey = "Drivetrain/Right Rear Absolute Counts";

        public static final String kDrivetrainLeftFrontRelativePositionKey = "Drivetrain/Left Front Relative Counts";
        public static final String kDrivetrainRightFrontRelativePositionKey = "Drivetrain/Right Front Relative Counts";
        public static final String kDrivetrainLeftRearRelativePositionKey = "Drivetrain/Left Rear Relative Counts";
        public static final String kDrivetrainRightRearRelativePositionKey = "Drivetrain/Right Rear Relative Counts";

        public static final String kDrivetrainLeftFrontAngleDegreesKey = "Drivetrain/Left Front Angle Degrees";
        public static final String kDrivetrainRightFrontAngleDegreesKey = "Drivetrain/Right Front Angle Degrees";
        public static final String kDrivetrainLeftRearAngleDegreesKey = "Drivetrain/Left Rear Angle Degrees";
        public static final String kDrivetrainRightRearAngleDegreesKey = "Drivetrain/Right Rear Angle Degrees";

        public static final String kDrivetrainLeftFrontVelocityErrorKey = "Drivetrain/Left Front Velocity Error";
        public static final String kDrivetrainRightFrontVelocityErrorKey = "Drivetrain/Right Front Velocity Error";
        public static final String kDrivetrainLeftRearVelocityErrorKey = "Drivetrain/Left Rear Velocity Error";
        public static final String kDrivetrainRightRearVelocityErrorKey = "Drivetrain/Right Rear Velocity Error";

        public static final String kDrivetrainLeftFrontZeroValueKey = "Drivetrain/Left Front Azimuth Zero Value";
        public static final String kDrivetrainRightFrontZeroValueKey = "Drivetrain/Right Front Azimuth Zero Value";
        public static final String kDrivetrainLeftRearZeroValueKey = "Drivetrain/Left Rear Azimuth Zero Value";
        public static final String kDrivetrainRightRearZeroValueKey = "Drivetrain/Right Rear Azimuth Zero Value";

        public static final String kDrivetrainSaveLeftFrontZeroKey = "Drivetrain/Save Left Front Azimuth Zero";
        public static final String kDrivetrainSaveRightFrontZeroKey = "Drivetrain/Save Right Front Azimuth Zero";
        public static final String kDrivetrainSaveLeftRearZeroKey = "Drivetrain/Save Left Rear Azimuth Zero";
        public static final String kDrivetrainSaveRightRearZeroKey = "Drivetrain/Save Right Rear Azimuth Zero";

        public static final String kDrivetrainAngularVelocity = "Drivetrain/Angular Velocity Dsec";

        public static final String kDriverForwardScaleKey = "Driver/Forward Scale";
        public static final String kDriverStrafeScaleKey = "Driver/Strafe Scale";
        public static final String kDriverYawScaleKey = "Driver/Yaw Scale";

        public static final String kIntakeVoltageKey = "Intake/Voltage";

        public static final String kTurnToAngleControllerKey = "Turn to Angle Controller";
        public static final String kTurnToAngleErrorDegrees = "Turn to Angle Error Deg";
        public static final String kTurnToAngleOmegaOutputKey = "Turn to Angle Omega Output";

        public static final String kLimelightAlignControllerKey = "Limelight Align Controller";
        public static final String kLimelightAlignErrorDegrees = "Limelight Align Error Deg";
        public static final String kLimelightAlignOmegaOutputKey = "Limelight Align Omega Output";


    }

    public static class LiveDashboardKeys {

        public static final String kLiveDashboardTableName = "Live_Dashboard";
        public static final String kRobotXKey = "robotX";
        public static final String kRobotYKey = "robotY";
        public static final String kRobotHeadingKey = "robotHeading";
        public static final String kIsFollowingPathKey = "isFollowingPath";
        public static final String kPathXKey = "pathX";
        public static final String kPathYKey = "pathY";
        public static final String kPathHeadingKey = "pathHeading";

    }

}
