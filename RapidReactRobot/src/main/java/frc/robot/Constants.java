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
        public static final boolean kIsInShooterTuningMode = false;

    }

    public static final class DriverPreferences {

        public static final class XboxConfigPorts {
            public static final int kDriveXboxControllerPort = 0;
            public static final int kManipulationXboxControllerPort = 1;
        }

        // Climbing
        public static final double kPivotManualOverrideDeadband = 0.1;
        public static final double kExtensionManualOverrideDeadband = 0.1;

        public static final double kFinishedClimbingRumbleValue = 1.0;

        public static final double kFaceDirectionToleranceDegrees = 2.0;

    }

    public static final class DioIDs {

        public static final int kRobotID1 = 8;
        public static final int kRobotID2 = 9;

    }

    public static final class PIDProfileSlots {

        public static final int kSpinningUpIndex = 0;
        public static final int kShootingIndex = 1;

    }

    public static final class Intake {

        public static final double rollUpTimeoutSeconds = 1.0;

    }

    public static final class Shooter {

        public static final double kAbsoluteMaximumVelocityNU = 15000;

        public static final double kDefaultVelocityRawUnits = 11500;
        public static final double kBarfVelocityRawUnits = 4500;
        public static final double kIdleVelocityRawUnits = 9000;
        public static final double kOuttakeVelocityRawUnits = -4500;

        public static final double kBarfHoodAngle = -0.672;

        // The tolerance to exit the spinning up state and enter the shooting state
        public static final double kSpinningUpToleranceRawUnits = 150;
        // The tolerance to maintain the shooting state
        public static final double kShootingToleranceRawUnits = 160;

        // Short period of time after the shoot button is released where the flywheels
        // continue rotating to ensure last few shots don't go amiss
        public static final double kSpinDownTimeSeconds = 0.25;

        public static final double kNudgeDownTimeSeconds = 0.15;

        // Time it takes for the hood to change positions
        public static final double kHoodTransitionTimeSeconds = 2.3;
        // The full range of angles for the hood
        public static final double kFullHoodAngleRange = 2.0;

    }

    public static final class Climber {

        public static final double kExtensionZeroingPercentOutput = -0.1;
        public static final double kFastExtensionZeroingPercentOutput = -0.2;

    }

    public static final class VisionConstants {

        public static final int kLEDsOff = 1;
        public static final int kLEDsOn = 3;

        public static final int kVisionMode = 0;
        public static final int kDriverMode = 1;

        public static final int kTxWindowSize = 1;
        public static final int kTyWindowSize = 5;

        public static final int kAlignmentPipeline = 0;

        public static final double kAlignmentToleranceDegrees = 1.5;
        public static final double kAlignmentTimeoutSeconds = 1.5;

    }

    public static final class SmartDashboardKeys {

        public static final String kDrivetrainSetModuleStatesKey = "Drivetrain/Set Module States";
        public static final String kDrivetrainSetpointAngleDegreesKey = "Drivetrain/Setpoint Angle Degrees";
        public static final String kDrivetrainSetpointVelocityKey = "Drivetrain/Setpoint Velocity MPS";

        public static final String kDrivetrainResetKey = "Drivetrain/Reset";

        public static final String kDrivetrainLeftFrontZeroValueKey = "Drivetrain/Left Front Azimuth Zero Value";
        public static final String kDrivetrainRightFrontZeroValueKey = "Drivetrain/Right Front Azimuth Zero Value";
        public static final String kDrivetrainLeftRearZeroValueKey = "Drivetrain/Left Rear Azimuth Zero Value";
        public static final String kDrivetrainRightRearZeroValueKey = "Drivetrain/Right Rear Azimuth Zero Value";

        public static final String kDrivetrainSaveLeftFrontZeroKey = "Drivetrain/Save Left Front Azimuth Zero";
        public static final String kDrivetrainSaveRightFrontZeroKey = "Drivetrain/Save Right Front Azimuth Zero";
        public static final String kDrivetrainSaveLeftRearZeroKey = "Drivetrain/Save Left Rear Azimuth Zero";
        public static final String kDrivetrainSaveRightRearZeroKey = "Drivetrain/Save Right Rear Azimuth Zero";

        public static final String kDrivetrainHeadingDegrees = "Drivetrain/Heading Degrees";
        public static final String kDrivetrainAngularVelocity = "Drivetrain/Angular Velocity DPS";
        public static final String kDrivetrainPitchDegrees = "Drivetrain/Pitch Degrees";
        public static final String kDrivetrainRollDegrees = "Drivetrain/Roll Degrees";

        public static final String kDrivetrainIsFieldRelativeKey = "Drivetrain/Is Field Relative";
        public static final String kDrivetrainIsPositionalRotationKey = "Drivetrain/Is Positional Rotation";

        public static final String kDriverForwardScaleKey = "Driver/Forward Scale";
        public static final String kDriverStrafeScaleKey = "Driver/Strafe Scale";
        public static final String kDriverYawScaleKey = "Driver/Yaw Scale";

        public static final String kDriverIsAlignedKey = "Driver/Is Aligned";
        public static final String kDriverIsMoneyShotKey = "Driver/Is Money Shot";

        public static final String kClimberPivotAngleFromVerticalKey = "Climber/Angle From Vertical Deg";
        public static final String kClimberPivotAngleFromHorizontalKey = "Climber/Angle From Horizontal Deg";

        public static final String kShooterCurrentTargetVelocityKey = "Shooter/Current Target Velocity NU";
        public static final String kShooterTuningSetpointVelocityNUKey = "Shooter/Tuning Setpoint Velocity NU";
        public static final String kShooterHoodPositionSetpointKey = "Shooter/Hood Position Setpoint";
        public static final String kShooterBallQualityAdditive = "Shooter/Ball Quality Additive";

        public static final String kDrivetrainPeriodicIOKey = "Drivetrain/Periodic IO";
        public static final String kIntakePeriodicIOKey = "Intake/Periodic IO";
        public static final String kConveyorPeriodicIOKey = "Conveyor/Periodic IO";
        public static final String kShooterPeriodicIOKey = "Shooter/Periodic IO";
        public static final String kClimberPeriodicIOKey = "Climber/Periodic IO";

        public static final String kLimelightAlignControllerKey = "Limelight Align Controller";
        public static final String kLimelightAlignErrorDegrees = "Limelight Align Error Deg";
        public static final String kLimelightAlignOmegaOutputKey = "Limelight Align Omega Output";
        public static final String kLimelightDistanceFeetKey = "Limelight Distance Feet";

        public static final String kTurnToAngleControllerKey = "Turn to Angle Controller";
        public static final String kTurnToAngleErrorDegreesKey = "Turn to Angle Error Deg";
        public static final String kTurnToAngleOmegaOutputKey = "Turn to Angle Omega Output";

        public static final String kLeftIntakePercentOutputKey = "Left Intake Percent Output";
        public static final String kRightIntakePercentOutputKey = "Right Intake Percent Output";

        public static final String kTrajectoryThetaPKey = "Trajectory/Theta P";

    }

    public static final class LiveDashboardKeys {

        public static final String kLiveDashboardTableName = "Live_Dashboard";
        public static final String kRobotXKey = "robotX";
        public static final String kRobotYKey = "robotY";
        public static final String kRobotHeadingKey = "robotHeading";
        public static final String kIsFollowingPathKey = "isFollowingPath";
        public static final String kPathXKey = "pathX";
        public static final String kPathYKey = "pathY";
        public static final String kPathHeadingKey = "pathHeading";

    }

    public static final class FieldConstants {

        public static final double kTargetHeightInches = 103.81;
        public static final double kSpinUpFlywheelDistanceFromHub = 10;

        public static final double kMoneyShotDistance = 8.1145;
        public static final double kMoneyShotTolerance = 0.25;

    }

}
