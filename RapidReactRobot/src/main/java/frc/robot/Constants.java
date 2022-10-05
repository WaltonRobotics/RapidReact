// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
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

        public static final boolean kIsInCompetition = true;
        public static final boolean kIsInTuningMode = true;
        public static final boolean kIsInShooterTuningMode = false;

    }

    public static final class DriverPreferences {

        public static final class XboxConfigPorts {
            public static final int kDriveXboxControllerPort = 0;
            public static final int kManipulationXboxControllerPort = 1;
        }

        public static final boolean kUseAzimuthDeadband = true;

        public static final boolean kMotionCorrectShooting = true;

        // Climbing
        public static final double kPivotManualOverrideDeadband = 0.1;
        public static final double kExtensionManualOverrideDeadband = 0.1;

        public static final double kFinishedClimbingRumbleValue = 1.0;

        public static final double kFaceDirectionToleranceDegrees = 2.0;

        public static final double kMaxShootOnTheMoveVelocity = 0.5;

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

        public static final double kDefaultHoodAngle = 0.247;

        public static final double kAbsoluteMaximumVelocityNU = 15500;

        public static final double kDefaultVelocityRawUnits = 9100;
        public static final double kBarfVelocityRawUnits = 4500;
        public static final double kAutonBarfVelocityRawUnits = 7000;
        public static final double kIdleVelocityRawUnits = 8700;
        public static final double kOuttakeVelocityRawUnits = -4500;

        public static final double kDefaultHighGoalHoodAngle = 0.5370843525179856;
        public static final double kDefaultHighGoalVelocity = 9080;
        public static final double kDefaultLowGoalHoodAngle = 0.2;
        public static final double kDefaultLowGoalVelocity = 6500;

        public static final double kBarfHoodAngle = 0;

        // The tolerance to exit the spinning up state and enter the shooting state
        public static final double kSpinningUpToleranceRawUnits = 70;
        // The tolerance to maintain the shooting state
        public static final double kShootingToleranceRawUnits = 150;

        // The tolerance needed to feed in the next ball
        public static final double kRecoveryToleranceRawUnits = 350;

        // Short period of time after the shoot button is released where the flywheels
        // continue rotating to ensure last few shots don't go amiss
        public static final double kSpinDownTimeSeconds = 0.25;

        public static final double kNudgeDownTimeSeconds = 0.15;

        // Limits
        public static final double kHoodLowerLimit = 0;

        // Time it takes for the hood to change positions
        public static final double kHoodTransitionTimeSeconds = 1.3;
        // The full range of angles for the hood
        public static final double kFullHoodAngleRange = 1;

        // NU conversion factors
        public static final double kFlywheelRadiusMeters = Units.inchesToMeters(1.25) + Units.inchesToMeters(0.5);
        public static final double kFlywheelNUToRPM = 600.0 / 2048.0 * 1.5;
        public static final double kFlywheelNUToMPS = kFlywheelNUToRPM / 60.0 * (2 * Math.PI * kFlywheelRadiusMeters);

        // Lower limit: 21.27642 deg from horizontal
        // Upper limit: 40.26936 deg from horizontal

        public static final double kHoodAngleToDegreesSlope = 40.26936 - 21.27642;
        public static final double kHoodAngleToDegreesIntercept = 21.27642;

    }

    public static final class Climber {

        public static final double kExtensionZeroingPercentOutput = -0.1;
        public static final double kFastExtensionZeroingPercentOutput = -0.2;
        public static final double kTransferPercentOutput = -0.2;

        public static final double kDefaultExtensionCruiseVelocity = 11250;
        public static final double kDefaultExtensionAcceleration = 9000;

        public static final double kDefaultPivotCruiseVelocity = 700;
        public static final double kDefaultPivotAcceleration = 800;

        public static final double kSlowPullUpExtensionCruiseVelocity = 10500;
        public static final double kSlowPullUpExtensionAcceleration = 8000;

        public static final double kPivotArmNudgeIncrementNU = 1137;

        public static final double kDeployHighBarArmsAngleDegrees = 57;

    }

    public static final class VisionConstants {

        public static final int kLEDsOff = 1;
        public static final int kLEDsOn = 3;

        public static final int kVisionMode = 0;
        public static final int kDriverMode = 1;

        public static final int kTxWindowSize = 1;
        public static final int kTyWindowSize = 5;

        public static final int kAlignmentPipeline = 0;

        public static final double kAutoAlignToleranceDegrees = 1.5;
        public static final double kShootingAlignmentToleranceDegrees = 2.25;
        public static final double kAlignmentTimeoutSeconds = 1.5;

        public static final boolean kUseOdometryBackup = false;

        public static final double kLimelightOffsetFeet = -0.11051343;

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

//        public static final String kDriverForwardScaleKey = "Driver/Forward Scale";
//        public static final String kDriverStrafeScaleKey = "Driver/Strafe Scale";
//        public static final String kDriverYawScaleKey = "Driver/Yaw Scale";

        public static final String kDriverIsAlignedKey = "Driver/Is Aligned";
        //        public static final String kDriverIsMoneyShotKey = "Driver/Is Money Shot";
        public static final String kDriverSelectedRungKey = "Driver/Selected Rung";

        public static final String kClimberPivotAngleFromVerticalKey = "Climber/Angle From Vertical Deg";
        public static final String kClimberPivotAngleFromHorizontalKey = "Climber/Angle From Horizontal Deg";

        public static final String kShooterCurrentTargetVelocityKey = "Shooter/Current Target Velocity NU";
        public static final String kShooterTuningSetpointVelocityNUKey = "Shooter/Tuning Setpoint Velocity NU";
        public static final String kShooterHoodPositionSetpointKey = "Shooter/Hood Position Setpoint";
        public static final String kShooterBallQualityAdditive = "Shooter/Ball Quality Additive";

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

        public static final String kClimberPivotCoastModeKey = "Climber Pivot Coast Mode";
        public static final String kClimberExtensionCoastModeKey = "Climber Extension Coast Mode";

        public static final String kClimberDeployHighBarArmsAngleKey = "Climber/Deploy High Bar Arms Angle";

        public static final String kAllianceColorKey = "Alliance Color";

        public static final String kFlywheelOnTargetKey = "Flywheel on target";

        public static final String kMaxShootOnTheMoveVelocityKey = "Max Shoot On The Move Velocity";

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

        public static final double kMoneyShotDistance = 8.05;
        public static final double kMoneyShotTolerance = 0.25;

        public static final Pose2d kCenterOfHubPose = new Pose2d(8.23, 4.11, Rotation2d.fromDegrees(0.0));

    }

    public static final class PathFollowing {

        public static final double kPathLookaheadTime = 0.25;  // seconds to look ahead along the path for steering 0.4
        public static final double kPathMinLookaheadDistance = Units.inchesToMeters(6.0);  // inches 24.0 (we've been using 3.0)

    }

}
