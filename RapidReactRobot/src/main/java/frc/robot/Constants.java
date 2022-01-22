// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.VelocityMeasPeriod;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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
    public static final class InputDevices {

        public static final int kLeftJoystickPort = 0;
        public static final int kRightJoystickPort = 1;
        public static final int kGamepadPort = 2;

    }

    public static final class SwerveDriveConfig {
        // TODO: verify diameter and run calibration
        // 500 cm calibration = actual / odometry
        public static final double kWheelDiameterInches = 3.0 * (584.0 / 501.0);

        // From: https://github.com/strykeforce/axis-config/
        public static final double kMaxSpeedMetersPerSecond = 3.889;

        public static final double kDistanceBetweenWheelsWidthWiseMeters = Units.inchesToMeters(15.0 + 1.0 / 16.0 + 1.761652 * 2.0);
        public static final double kDistanceBetweenWheelsLengthWiseMeters = Units.inchesToMeters(16.0 + 17.0 / 32.0 + 1.761652 * 2.0);
        //Luke says hi

        public static final double kMaxOmega =
                (kMaxSpeedMetersPerSecond / Math.hypot(kDistanceBetweenWheelsLengthWiseMeters / 2.0,
                        kDistanceBetweenWheelsWidthWiseMeters / 2.0))
                        / 2.0; // wheel locations below

        // From: https://github.com/strykeforce/axis-config/
        static final double kDriveMotorOutputGear = 22;
        static final double kDriveInputGear = 48;
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
            driveConfig.slot0.kP = 0.045;
            driveConfig.slot0.kI = 0.0005;
            driveConfig.slot0.kD = 0.000;
            driveConfig.slot0.kF = 0.047;
            driveConfig.slot0.integralZone = 500;
            driveConfig.slot0.maxIntegralAccumulator = 75_000;
            driveConfig.slot0.allowableClosedloopError = 0;
            //driveConfig.velocityMeasurementPeriod = VelocityMeasPeriod.Period_100Ms;
            driveConfig.velocityMeasurementWindow = 64;
            driveConfig.voltageCompSaturation = 12;
            return driveConfig;
        }
    }

    public static final class DioIDs {

        public static final int kRobotID1 = 8;
        public static final int kRobotID2 = 9;

    }

    public static class Field{
        public static final int kTargetHeightInches = 104;
    }
    public static class Limelight {

        public static final int kLEDsOff = 1;
        public static final int kLEDsOn = 3;

        public static final int kVisionMode = 0;
        public static final int kDriverMode = 1;

        public static final int kTxWindowSize = 1;
        public static final int kTyWindowSize = 5;
        public static final int kCamtranWindowSize = 5;

        public static final int kAlignmentPipeline = 0;
        public static final int kPnPPipeline = 1;

        public static final double kMaximumLEDWaitTimeSeconds = 0.5;

    }

    public static class SmartDashboardKeys{
        public static final String kRobotIdentifierKey = "Robot/Robot Identifier";

        public static final String kDrivetrainLeftVelocityPKey = "Drivetrain/Left Velocity P";
        public static final String kDrivetrainRightVelocityPKey = "Drivetrain/Right Velocity P";
        public static final String kDrivetrainAngularVelocityKey = "Drivetrain/Angular Velocity Deg/s";
        public static final String kDrivetrainHeadingKey = "Drivetrain/Heading (deg)";
        public static final String kDrivetrainLeftPositionKey = "Drivetrain/Left Encoder Position Meters";
        public static final String kDrivetrainRightPositionKey = "Drivetrain/Right Encoder Position Meters";
        public static final String kDrivetrainLeftVelocityKey = "Drivetrain/Left Encoder Velocity Meters/s";
        public static final String kDrivetrainRightVelocityKey = "Drivetrain/Right Encoder Velocity Meters/s";
        public static final String kDrivetrainTuningOpenLoopRampRateKey = "Drivetrain/Tuning Open Loop Ramp Rate";

        public static final String kTurnToAngleHeadingSetpointKey = "Turn to Angle/Heading Setpoint Deg";
        public static final String kTurnToAnglePositionErrorKey = "Turn to Angle/Position Error Deg";
        public static final String kTurnToAngleVelocityErrorKey = "Turn to Angle/Velocity Error Deg/s";
        public static final String kTurnToAngleRateKey = "Turn to Angle/Rate";
        public static final String kTurnToAnglePKey = "Turn to Angle/Turn P";
        public static final String kTurnToAngleIKey = "Turn to Angle/Turn I";
        public static final String kTurnToAngleDKey = "Turn to Angle/Turn D";

        public static final String kDriveStraightDistanceAverageKey = "Drive Straight/Distance Average";
        public static final String kDriveStraightForwardPKey = "Drive Straight/Forward P";
        public static final String kDriveStraightForwardIKey = "Drive Straight/Forward I";
        public static final String kDriveStraightForwardDKey = "Drive Straight/Forward D";
        public static final String kDriveStraightHeadingPKey = "Drive Straight/Heading P";
        public static final String kDriveStraightHeadingIKey = "Drive Straight/Heading I";
        public static final String kDriveStraightHeadingDKey = "Drive Straight/Heading D";
        public static final String kDriveStraightForwardErrorKey = "Drive Straight/Forward Error";
        public static final String kDriveStraightForwardRateKey = "Drive Straight/Forward Rate";
        public static final String kDriveStraightHeadingErrorKey = "Drive Straight/Heading Error";
        public static final String kDriveStraightTurnRateKey = "Drive Straight/Turn Rate";

        public static final String kShooterMeasurementPeriodKey = "Shooter/Measurement Period";
        public static final String kShooterMeasurementWindowKey = "Shooter/Measurement Window";
        public static final String kShooterErrorRawUnitsKey = "Shooter/Error Raw Units";
        public static final String kShooterErrorRPSKey = "Shooter/Error RPS";
        public static final String kShooterErrorInchesKey = "Shooter/Error Inches Per Sec";
        public static final String kShooterFlywheelVelocityKey = "Shooter/Flywheel Velocity Raw Units";
        public static final String kShooterCurrentSetpointRawUnitsKey = "Shooter/Current Setpoint Raw Units";
        public static final String kShooterTuningSetpointRawUnitsKey = "Shooter/Tuning Setpoint Raw Units";
        public static final String kShooterLimelightDistanceFeetKey = "Shooter/Limelight Distance Feet";
        public static final String kShooterIsAdjustableHoodUpKey = "Shooter/Is Adjustable Hood Up";

        public static final String kIntakeIntakingDutyCycleKey = "Intake/Tuning Intaking Duty Cycle";

        public static final String kConveyorFrontSensorStateKey = "Conveyor/Front Sensor State";
        public static final String kConveyorBackSensorStateKey = "Conveyor/Back Sensor State";
        public static final String kConveyorBallCountKey = "Conveyor/Ball Count";
        public static final String kConveyorTuningNudgeTime = "Conveyor/Tuning Nudge Time Seconds";

        public static final String kTurretForwardLimitStateKey = "Turret/Forward Limit State";
        public static final String kTurretRobotRelativeHeadingRawUnitsKey = "Turret/Robot-relative Heading Raw Units";
        public static final String kTurretRobotRelativeHeadingDegreesKey = "Turret/Robot-relative Heading Degrees";
        public static final String kTurretFieldRelativeHeadingDegreesKey = "Turret/Field-relative Heading Degrees";
        public static final String kTurretAngularVelocityRawUnitsKey = "Turret/Angular Velocity Raw Units";
        public static final String kTurretControlStateKey = "Turret/Control State";
        public static final String kTurretSetpointKey = "Turret/Setpoint";
        public static final String kTurretClosedLoopErrorDegreesKey = "Turret/Closed Loop Error Degrees";
        public static final String kTurretIsHomedForClimbingKey = "Turret/Is Homed For Climbing";
        public static final String kTurretIsAlignedKey = "Turret/Is Aligned";

        public static final String kClimberIsUnlockedKey = "Climber/Is Unlocked";
        public static final String kClimberIsDeployedKey = "Climber/Is Deployed";

        public static final String kProMicroLEDWriteMessageKey = "Pro Micro/LED Write Message";
        public static final String kProMicroPixyCamReadMessageKey = "Pro Micro/PixyCam Read Message";

        public static final String kNormalScaleFactorKey = "Driver Preferences/Normal Scale Factor";
        public static final String kTurboScaleFactorKey = "Driver Preferences/Turbo Scale Factor";
        public static final String kCurvatureTurnSensitivityKey = "Driver Preferences/Turn Sensitivity";

        public static final String kLimelightSolvePnPXInchesKey = "Limelight Solve PnP/X Inches";
        public static final String kLimelightSolvePnPYInchesKey = "Limelight Solve PnP/Y Inches";
        public static final String kLimelightSolvePnPZInchesKey = "Limelight Solve PnP/Z Inches";
        public static final String kLimelightSolvePnPPitchDegreesKey = "Limelight Solve PnP/Pitch Degrees";
        public static final String kLimelightSolvePnPYawDegreesKey = "Limelight Solve PnP/Yaw Degrees";
        public static final String kLimelightSolvePnPRollDegreesKey = "Limelight Solve PnP/Roll Degrees";

    }
}
