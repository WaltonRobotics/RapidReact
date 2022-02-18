// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

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

    public static final class PIDProfileSlots {

        public static final int kShooterDefaultIndex = 0;

    }

    public static final class Climber {

        public static final double kExtensionZeroingPercentOutput = -0.2;

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

        public static final String kDrivetrainLeftFrontZeroValueKey = "Drivetrain/Left Front Azimuth Zero Value";
        public static final String kDrivetrainRightFrontZeroValueKey = "Drivetrain/Right Front Azimuth Zero Value";
        public static final String kDrivetrainLeftRearZeroValueKey = "Drivetrain/Left Rear Azimuth Zero Value";
        public static final String kDrivetrainRightRearZeroValueKey = "Drivetrain/Right Rear Azimuth Zero Value";

        public static final String kDrivetrainSaveLeftFrontZeroKey = "Drivetrain/Save Left Front Azimuth Zero";
        public static final String kDrivetrainSaveRightFrontZeroKey = "Drivetrain/Save Right Front Azimuth Zero";
        public static final String kDrivetrainSaveLeftRearZeroKey = "Drivetrain/Save Left Rear Azimuth Zero";
        public static final String kDrivetrainSaveRightRearZeroKey = "Drivetrain/Save Right Rear Azimuth Zero";

        public static final String kClimberPivotAngleFromVertical = "Climber/Angle From Vertical Deg";
        public static final String kClimberPivotAngleFromHorizontal = "Climber/Angle From Horizontal Deg";

        public static final String kDriverForwardScaleKey = "Driver/Forward Scale";
        public static final String kDriverStrafeScaleKey = "Driver/Strafe Scale";
        public static final String kDriverYawScaleKey = "Driver/Yaw Scale";

        public static final String kDrivetrainPeriodicIOKey = "Drivetrain/Periodic IO";
        public static final String kIntakePeriodicIOKey = "Intake/Periodic IO";
        public static final String kConveyorPeriodicIOKey = "Conveyor/Periodic IO";
        public static final String kShooterPeriodicIOKey = "Shooter/Periodic IO";
        public static final String kClimberPeriodicIOKey = "Climber/Periodic IO";

        public static final String kDrivetrainAngularVelocity = "Drivetrain/Angular Velocity Dsec";

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
