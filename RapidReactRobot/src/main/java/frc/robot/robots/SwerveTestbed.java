package frc.robot.robots;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.config.SmartMotionConstants;

public class SwerveTestbed extends WaltRobot {

    @Override
    public void configDrivetrain() {
        for (int i = 0; i < 4; i++) {
            SmartMotionConstants azimuthConfig = new SmartMotionConstants();

            azimuthConfig.velocityPID = new PIDController(0.003198, 0.00006396, 0.0);
            azimuthConfig.iZone = 0.0;
            azimuthConfig.feedforward = 0.00997776;
            azimuthConfig.minOutput = -1;
            azimuthConfig.maxOutput = 1;
            azimuthConfig.maxVelocity = 120;
            azimuthConfig.minOutputVelocity = 0;
            azimuthConfig.maxAccel = 100;
            azimuthConfig.allowedClosedLoopError = 0;

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

            drivetrainConfig.azimuthControllerConfigs[i] = azimuthConfig;
            drivetrainConfig.driveControllerConfigs[i] = driveConfig;
        }

        drivetrainConfig.azimuthControllerIDs = new int[] { 1, 2, 3, 4 };
        drivetrainConfig.driveControllerIDs = new int[] { 11, 12, 13, 14 };
        drivetrainConfig.absoluteEncoderChannels = new int[] { 1, 2, 3, 4 };

        drivetrainConfig.azimuthControllerInversions = new boolean[]{ true, true, true, true };
        drivetrainConfig.driveControllerInversions = new boolean[]{ false, true, true, false };
        drivetrainConfig.absoluteEncoderInversions = new boolean[]{ true, true, true, true };

        drivetrainConfig.kRelativeEncoderRotationsPerTick = 1.0 / (5.33 * 12.0);

        final double kDistanceBetweenWheelsWidthWiseMeters = Units.inchesToMeters(15.0 + 1.0 / 16.0 + 1.761652 * 2.0); // 18.586 in
        final double kDistanceBetweenWheelsLengthWiseMeters = Units.inchesToMeters(16.0 + 17.0 / 32.0 + 1.761652 * 2.0); // 20.055 in

        final double x = kDistanceBetweenWheelsLengthWiseMeters / 2.0; // front-back, was ROBOT_LENGTH
        final double y = kDistanceBetweenWheelsWidthWiseMeters / 2.0; // left-right, was ROBOT_WIDTH

        Translation2d[] locs = new Translation2d[4];
        locs[0] = new Translation2d(x, y); // left front
        locs[1] = new Translation2d(x, -y); // right front
        locs[2] = new Translation2d(-x, y); // left rear
        locs[3] = new Translation2d(-x, -y); // right rear

        drivetrainConfig.wheelLocationMeters = locs;

        drivetrainConfig.kWheelDiameterInches = 3.0;

        drivetrainConfig.kMaxSpeedMetersPerSecond = 3.889;
        drivetrainConfig.kMaxOmega = (drivetrainConfig.kMaxSpeedMetersPerSecond / Math.hypot(kDistanceBetweenWheelsLengthWiseMeters / 2.0,
                kDistanceBetweenWheelsWidthWiseMeters / 2.0))
                / 2.0; // wheel locations below;

        drivetrainConfig.kTalonConfigTimeout = 10;

        final double kDriveMotorOutputGear = 12;
        final double kDriveInputGear = 21;
        final double kBevelInputGear = 15;
        final double kBevelOutputGear = 45;

        drivetrainConfig.kDriveGearRatio =
                (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);

        final double kTranslationalP = 6.0;
        final double kTranslationalD = kTranslationalP / 100.0;

        drivetrainConfig.xController = new PIDController(kTranslationalP, 0.0, kTranslationalD);
        drivetrainConfig.yController = new PIDController(kTranslationalP, 0.0, kTranslationalD);
        drivetrainConfig.thetaController =
                new ProfiledPIDController(
                        3.0,
                        0,
                        0,
                        new TrapezoidProfile.Constraints(drivetrainConfig.kMaxOmega / 2.0, 3.14));
    }

    @Override
    public void configIntake() {

    }

    @Override
    public void configConveyor() {

    }

    @Override
    public void configShooter() {

    }

    @Override
    public void configClimber() {

    }

}
