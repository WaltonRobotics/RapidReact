package frc.robot.robots;

import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.config.*;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.util.AccelerationLimiter;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.util.interpolation.InterpolatingTreeMap;

import java.util.HashMap;

import static frc.robot.Constants.Climber.kDefaultExtensionAcceleration;
import static frc.robot.Constants.Climber.kDefaultExtensionCruiseVelocity;
import static frc.robot.subsystems.Climber.ClimberExtensionLimits.*;
import static frc.robot.subsystems.Climber.ClimberExtensionPosition.*;
import static frc.robot.subsystems.Climber.ClimberPivotLimits.*;
import static frc.robot.subsystems.Climber.ClimberPivotPosition.*;

public class PracticeRapidReact extends WaltRobot {

    // CAN IDs
    // 1: Azimuth front-left
    // 2: Azimuth front-right
    // 3: Azimuth rear-left
    // 4: Azimuth rear-right
    // 5: Left intake rollers
    // 6: Right intake rollers
    // 7: Transport conveyor
    // 8: Feed conveyor
    // 9: Flywheel master
    // 10: Flywheel slave
    // 11: Drive front-left
    // 12: Drive front-right
    // 13: Drive rear-left
    // 14: Drive rear-right
    // 15: Climber pivot
    // 16: Climber extension

    // Drivetrain constants
    private final PIDController azimuthController = new PIDController(15.0 / 4096.0, 0.0, 0.0);
    private final TalonFXConfiguration[] driveControllerConfigs = new TalonFXConfiguration[4];

    // Bumper-bumper length: 33.489 in
    // Bumper-bumper width: 26.458 in
    private final double kDistanceBetweenWheelsWidthWiseMeters =
            Units.inchesToMeters(13.173279);

    private final double kDistanceBetweenWheelsLengthWiseMeters =
            Units.inchesToMeters(20.173279);

    private final Translation2d[] wheelLocationMeters = new Translation2d[4];

    private final double kTranslationalP = 6.0;
    private final double kTranslationalD = 0.02;

    private final double kMaxSpeedMetersPerSecond = 3.889;
    private final double kMaxOmega = (kMaxSpeedMetersPerSecond / Math.hypot(kDistanceBetweenWheelsLengthWiseMeters / 2.0,
            kDistanceBetweenWheelsWidthWiseMeters / 2.0))
            / 2.0;

    private final PIDController xController = new PIDController(kTranslationalP, 0.0, kTranslationalD);
    private final PIDController yController = new PIDController(kTranslationalP, 0.0, kTranslationalD);
    private final ProfiledPIDController thetaController =
            new ProfiledPIDController(
                    3.5,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(kMaxOmega / 2.0, 3.14));

    private final PIDController faceDirectionController = new PIDController(0.09, 0, 0);
    private final PIDController autoAlignController = new PIDController(0.05, 0, 0);
    private final ProfiledPIDController turnToAngleController = new ProfiledPIDController
            (0.05, 0.015, 0, new TrapezoidProfile.Constraints(
                    Math.toDegrees(kMaxOmega / 1.1), 360.0));

    // Shooter constants
    private final TalonFXConfiguration flywheelMasterTalonConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration flywheelSlaveTalonConfig = new TalonFXConfiguration();

    private final double[][] lowGoalMap = {
            // Actual measured distance, Limelight distance, hood angle, velocity
            {0, 3.879, 0, 5000},
            {0, 6.456, 0, 6200},
    };

    private final double[][] highGoalMap = {
            // Actual measured distance (to front bumper) inches, Limelight distance, hood angle, velocity
            {-1, 5.139874, 0, 8800},
            {-1, 6.110874, 0.2, 8850},
            {-1, 7.051874, 0.4, 9000},
            {-1, 8.163874, 0.7, 9177},
            {-1, 8.979874, 0.75, 9370},
            {-1, 9.876874, 0.75, 9600},
            {-1, 10.961874, 0.75, 10050},
            {-1, 11.948874, 0.75, 10400},
            {-1, 12.873874, 0.75, 10679},
            {-1, 13.778874, 0.9, 11000},
            {-1, 14.951874, 1.0, 11390},
            {-1, 15.857874, 1.0, 11718},
            {-1, 16.678874, 1.0, 12001},
            {-1, 17.840874, 1.0, 12403},
            {-1, 19.388874, 1.0, 13000},
            {-1, 20.253874, 1.0, 13350},
    };

    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> lowGoalFlywheelVelocityMap
            = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> lowGoalHoodAngleMap
            = new InterpolatingTreeMap<>();

    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> highGoalFlywheelVelocityMap
            = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> highGoalHoodAngleMap
            = new InterpolatingTreeMap<>();

    // Climber constants
    private final TalonFXConfiguration pivotControllerTalonConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration extensionControllerTalonConfig = new TalonFXConfiguration();

    private final HashMap<Climber.ClimberPivotLimits, LimitPair> climberPivotLimits = new HashMap<>(5);
    private final HashMap<Climber.ClimberPivotPosition, Target> climberPivotTargets = new HashMap<>(10);

    private final HashMap<Climber.ClimberExtensionLimits, LimitPair> climberExtensionLimits = new HashMap<>(3);
    private final HashMap<Climber.ClimberExtensionPosition, Target> climberExtensionTargets = new HashMap<>(7);

    public PracticeRapidReact() {
        configAll();
    }

    @Override
    public void configDrivetrain() {
        for (int i = 0; i < 4; i++) {
            TalonFXConfiguration driveConfig = new TalonFXConfiguration();
            driveConfig.supplyCurrLimit.currentLimit = 0.04;
            driveConfig.supplyCurrLimit.triggerThresholdCurrent = 45;
            driveConfig.supplyCurrLimit.triggerThresholdTime = 40;
            driveConfig.supplyCurrLimit.enable = true;
            driveConfig.slot0.kP = 0.00096971;
            driveConfig.slot0.kI = 0;
            driveConfig.slot0.kD = 0;
            driveConfig.slot0.kF = 0;
//            driveConfig.slot0.kP = 0.045;
//            driveConfig.slot0.kI = 0.0005;
//            driveConfig.slot0.kD = 0.000;
//            driveConfig.slot0.kF = 0.047;
            driveConfig.slot0.integralZone = 750;
            driveConfig.slot0.maxIntegralAccumulator = 75_000;
            driveConfig.slot0.allowableClosedloopError = 0;
            driveConfig.velocityMeasurementPeriod = SensorVelocityMeasPeriod.Period_50Ms;
            driveConfig.velocityMeasurementWindow = 32;
            driveConfig.voltageCompSaturation = 12;

            driveControllerConfigs[i] = driveConfig;
        }

        final double x = kDistanceBetweenWheelsLengthWiseMeters / 2.0; // front-back, was ROBOT_LENGTH
        final double y = kDistanceBetweenWheelsWidthWiseMeters / 2.0; // left-right, was ROBOT_WIDTH

        wheelLocationMeters[0] = new Translation2d(x, y); // left front
        wheelLocationMeters[1] = new Translation2d(x, -y); // right front
        wheelLocationMeters[2] = new Translation2d(-x, y); // left rear
        wheelLocationMeters[3] = new Translation2d(-x, -y); // right rear

        azimuthController.setTolerance(1);

        turnToAngleController.enableContinuousInput(-180.0, 180.0);
        turnToAngleController.setTolerance(1.5, 1.0);
        faceDirectionController.enableContinuousInput(-180, 180);
        autoAlignController.enableContinuousInput(-180, 180);

        drivetrainConfig = new DrivetrainConfig() {
            @Override
            public PIDController getAzimuthPositionalPID() {
                return azimuthController;
            }

            @Override
            public TalonFXConfiguration[] getDriveControllerConfigs() {
                return driveControllerConfigs;
            }

            @Override
            public int[] getAzimuthControllerIDs() {
                return new int[]{1, 2, 3, 4};
            }

            @Override
            public int[] getDriveControllerIDs() {
                return new int[]{11, 12, 13, 14};
            }

            @Override
            public int[] getAbsoluteEncoderChannels() {
                return new int[]{0, 1, 2, 3};
            }

            @Override
            public boolean[] getAzimuthControllerInversions() {
                return new boolean[]{true, true, true, true};
            }

            @Override
            public boolean[] getDriveControllerInversions() {
                return new boolean[]{true, true, true, true};
            }

            @Override
            public boolean[] getAbsoluteEncoderInversions() {
                return new boolean[]{true, true, true, true};
            }

            @Override
            public double getRelativeEncoderRotationsPerTick() {
                return 1.0 / 64.0;
            }

            @Override
            public double getMaxSpeedMetersPerSecond() {
                return kMaxSpeedMetersPerSecond;
            }

            @Override
            public double getMaxOmega() {
                return kMaxOmega;
            }

            @Override
            public double getMaxFaceDirectionOmega() {
                return kMaxOmega;
            }

            @Override
            public double getDriveMetersPerNU() {
                // GR / CPR * Circumference
                return Units.feetToMeters(5.0) / 69932.75;
            }

            @Override
            public Translation2d[] getWheelLocationMeters() {
                return wheelLocationMeters;
            }

            @Override
            public AccelerationLimiter getXLimiter() {
                return new AccelerationLimiter(kMaxSpeedMetersPerSecond / 0.4,
                        kMaxSpeedMetersPerSecond / 0.4);
            }

            @Override
            public AccelerationLimiter getYLimiter() {
                return new AccelerationLimiter(kMaxSpeedMetersPerSecond / 0.4,
                        3.75);
            }

            @Override
            public AccelerationLimiter getOmegaLimiter() {
                return new AccelerationLimiter(kMaxOmega / 0.4,
                        kMaxOmega / 0.4);
            }

            @Override
            public PIDController getXController() {
                return xController;
            }

            @Override
            public PIDController getYController() {
                return yController;
            }

            @Override
            public ProfiledPIDController getThetaController() {
                return thetaController;
            }

            @Override
            public PIDController getFaceDirectionController() {
                return faceDirectionController;
            }

            @Override
            public PIDController getAutoAlignController() {
                return autoAlignController;
            }

            @Override
            public double getMinTurnOmega() {
                return 0.05;
            }

            @Override
            public ProfiledPIDController getTurnToAngleController() {
                return turnToAngleController;
            }

            @Override
            public double getClimbingMaxMetersPerSecond() {
                return kMaxSpeedMetersPerSecond / 3.;
            }

            @Override
            public double getClimbingMaxOmega() {
                return kMaxOmega / 3.;
            }
        };
    }

    @Override
    public void configIntake() {
        intakeConfig = new IntakeConfig() {
            @Override
            public MotorConfig getLeftIntakeControllerConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 5;
                    }

                    @Override
                    public boolean isInverted() {
                        return true;
                    }
                };
            }

            @Override
            public MotorConfig getRightIntakeControllerConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 6;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public int getLeftIntakeSolenoidChannel() {
                return 0;
            }

            @Override
            public int getRightIntakeSolenoidChannel() {
                return 3;
            }

            @Override
            public double getLeftIntakePercentOutput() {
                return 0.35;
            }

            @Override
            public double getRightIntakePercentOutput() {
                return 0.35; // 0.35
            }

            @Override
            public double getLeftOuttakePercentOutput() {
                return -0.45;
            }

            @Override
            public double getRightOuttakePercentOutput() {
                return -0.45; // -0.35
            }
        };
    }

    @Override
    public void configConveyor() {
        conveyorConfig = new ConveyorConfig() {
            @Override
            public MotorConfig getTransportControllerConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 7;
                    }

                    @Override
                    public boolean isInverted() {
                        return true;
                    }
                };
            }

            @Override
            public MotorConfig getFeedControllerConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 8;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public double getTransportIntakePercentOutput() {
                return 0.5;
            }

            @Override
            public double getTransportOuttakePercentOutput() {
                return -0.65;
            }

            @Override
            public double getFeedOuttakePercentOutput() {
                return -0.65;
            }

            @Override
            public double getTransportShootPercentOutput() {
                return 1;
            }

            @Override
            public double getFeedShootPercentOutput() {
                return 1;
            }
        };
    }

    @Override
    public void configShooter() {
        flywheelMasterTalonConfig.voltageCompSaturation = 12.0;
        flywheelSlaveTalonConfig.voltageCompSaturation = 12.0;

        // Spinning up profile
        flywheelMasterTalonConfig.slot0.kF = 0.05;
        flywheelMasterTalonConfig.slot0.kP = 0.00085;
        flywheelMasterTalonConfig.slot0.kI = 1.38282776E-05;
        flywheelMasterTalonConfig.slot0.kD = 0;
        flywheelMasterTalonConfig.slot0.allowableClosedloopError = 0;
        flywheelMasterTalonConfig.slot0.integralZone = 300;
        flywheelMasterTalonConfig.slot0.maxIntegralAccumulator = 300000;
        flywheelMasterTalonConfig.slot0.closedLoopPeakOutput = 1.0;

        // Shooting profile
        flywheelMasterTalonConfig.slot1.kF = 0.05;
        flywheelMasterTalonConfig.slot1.kP = 0.00088;
        flywheelMasterTalonConfig.slot1.kI = 1.38282776E-05;
        flywheelMasterTalonConfig.slot1.kD = 0;
        flywheelMasterTalonConfig.slot1.allowableClosedloopError = 0;
        flywheelMasterTalonConfig.slot1.integralZone = 300;
        flywheelMasterTalonConfig.slot1.maxIntegralAccumulator = 300000;
        flywheelMasterTalonConfig.slot1.closedLoopPeakOutput = 1.0;

        shooterConfig = new ShooterConfig() {
            @Override
            public MotorConfig getFlywheelMasterControllerMotorConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 9;
                    }

                    @Override
                    public boolean isInverted() {
                        return true;
                    }
                };
            }

            @Override
            public MotorConfig getFlywheelSlaveControllerMotorConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 10;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public TalonFXConfiguration getFlywheelMasterControllerTalonConfig() {
                return flywheelMasterTalonConfig;
            }

            @Override
            public TalonFXConfiguration getFlywheelSlaveControllerTalonConfig() {
                return flywheelSlaveTalonConfig;
            }

            @Override
            public MotorConfig getAdjustableHoodServoConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 2;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public double getLimelightMountingHeightInches() {
                return 42.2815;
            }

            @Override
            public double getLimelightMountingAngleDegrees() {
                return 32;
            }

            @Override
            public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getFlywheelVelocityMap(Shooter.AimTarget target) {
                if (target == Shooter.AimTarget.LOW_GOAL) {
                    return lowGoalFlywheelVelocityMap;
                }

                return highGoalFlywheelVelocityMap;
            }

            @Override
            public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getHoodAngleMap(Shooter.AimTarget target) {
                if (target == Shooter.AimTarget.LOW_GOAL) {
                    return lowGoalHoodAngleMap;
                }

                return highGoalHoodAngleMap;
            }
        };
    }

    @Override
    public void configClimber() {
        pivotControllerTalonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                true, 35, 35, 1);
        pivotControllerTalonConfig.voltageCompSaturation = 12.0;
        pivotControllerTalonConfig.forwardSoftLimitEnable = false;
        pivotControllerTalonConfig.reverseSoftLimitEnable = false;

        // Motion Magic slot
        pivotControllerTalonConfig.slot0.kF = 0.11366667;
        pivotControllerTalonConfig.slot0.kP = 1.0;
        pivotControllerTalonConfig.slot0.kI = 1.0 / 100.0;
        pivotControllerTalonConfig.slot0.kD = 0;
        pivotControllerTalonConfig.slot0.allowableClosedloopError = 0;
        pivotControllerTalonConfig.slot0.integralZone = 300; // 750
        pivotControllerTalonConfig.slot0.maxIntegralAccumulator = 250_000;
        pivotControllerTalonConfig.slot0.closedLoopPeakOutput = 1.0;
        pivotControllerTalonConfig.motionCruiseVelocity = 700;
        pivotControllerTalonConfig.motionAcceleration = 600;
        pivotControllerTalonConfig.motionCurveStrength = 3;

        extensionControllerTalonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                true, 35, 40, 1);
        extensionControllerTalonConfig.voltageCompSaturation = 12.0;
        extensionControllerTalonConfig.forwardSoftLimitEnable = true;
        extensionControllerTalonConfig.reverseSoftLimitEnable = true;

        // Motion Magic slot
        extensionControllerTalonConfig.slot0.kF = 0.04972917;
        extensionControllerTalonConfig.slot0.kP = 0.5;
        extensionControllerTalonConfig.slot0.kI = 0.5 / 100.0;
        extensionControllerTalonConfig.slot0.kD = 0;
        extensionControllerTalonConfig.slot0.allowableClosedloopError = 0;
        extensionControllerTalonConfig.slot0.integralZone = 100;
        extensionControllerTalonConfig.slot0.maxIntegralAccumulator = 0;
        extensionControllerTalonConfig.slot0.closedLoopPeakOutput = 1.0;
        extensionControllerTalonConfig.motionCruiseVelocity = kDefaultExtensionCruiseVelocity;
        extensionControllerTalonConfig.motionAcceleration = kDefaultExtensionAcceleration;
        extensionControllerTalonConfig.motionCurveStrength = 3;

        climberConfig = new ClimberConfig() {
            @Override
            public HashMap<Climber.ClimberExtensionLimits, LimitPair> getClimberExtensionLimits() {
                return climberExtensionLimits;
            }

            @Override
            public HashMap<Climber.ClimberExtensionPosition, Target> getClimberExtensionTargets() {
                return climberExtensionTargets;
            }

            @Override
            public double getVerticalReferenceAbsoluteCounts() {
                return 1001;
            }

            @Override
            public double getMaxGravityFeedForward() {
                return 0;
            }

            @Override
            public HashMap<Climber.ClimberPivotLimits, LimitPair> getClimberPivotLimits() {
                return climberPivotLimits;
            }

            @Override
            public HashMap<Climber.ClimberPivotPosition, Target> getClimberPivotTargets() {
                return climberPivotTargets;
            }

            @Override
            public MotorConfig getPivotControllerMotorConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 15;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public MotorConfig getExtensionControllerMotorConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 16;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public TalonFXConfiguration getPivotControllerTalonConfig() {
                return pivotControllerTalonConfig;
            }

            @Override
            public TalonFXConfiguration getExtensionControllerTalonConfig() {
                return extensionControllerTalonConfig;
            }

            @Override
            public AbsoluteEncoderConfig getPivotAngleAbsoluteEncoderConfig() {
                return new AbsoluteEncoderConfig() {
                    @Override
                    public double getDistancePerRotation() {
                        return 1024;
                    }

                    @Override
                    public int getChannel() {
                        return 4;
                    }

                    @Override
                    public boolean isInverted() {
                        return true;
                    }
                };
            }

            @Override
            public int getLeftExtensionLowerLimitChannel() {
                return 5;
            }

            @Override
            public int getRightExtensionLowerLimitChannel() {
                return 6;
            }

            @Override
            public int getClimberLockSolenoidChannel() {
                return 2;
            }

            @Override
            public int getClimberDiscBrakeSolenoidChannel() {
                return 1;
            }

            @Override
            public double getManualPivotPercentOutputLimit() {
                return 0.35;
            }

            @Override
            public double getExtensionManualPercentOutputLimit() {
                return 0.3;
            }

            @Override
            public double getAbsoluteCountsToIntegratedCountsFactor() {
                return (200.0 * 2048.0) / 1024.0;
            }
        };
    }

    @Override
    public void defineLimits() {
        climberPivotLimits.put(PIVOT_STOWED, new LimitPair(956, 1006));
        climberPivotLimits.put(PIVOT_FULL_ROM, new LimitPair(969, 1065));
        climberPivotLimits.put(PIVOT_PULL_UP_TO_MID_BAR, new LimitPair(969, 1065));
        climberPivotLimits.put(PIVOT_PULL_UP_TO_HIGH_BAR, new LimitPair(969, 1065));
        climberPivotLimits.put(PIVOT_PULL_UP_TO_TRANSFER_HIGH_BAR, new LimitPair(969, 1065));
        climberPivotLimits.put(PIVOT_PULL_UP_TO_TRAVERSAL_BAR, new LimitPair(969, 1065));

        climberExtensionLimits.put(STOWED, new LimitPair(5000, 8000));
        climberExtensionLimits.put(EXTENSION_FULL_ROM, new LimitPair(5000, 470081));
        climberExtensionLimits.put(MID_BAR_POSITION_FIXED_ARM, new LimitPair(5000, 470081));
        climberExtensionLimits.put(HIGH_BAR_TRANSFER_TO_FIXED_ARM, new LimitPair(5000, 470081));
    }

    @Override
    public void defineTargets() {
        for (double[] tableRow : lowGoalMap) {
            double distance = tableRow[1];
            double hoodAngle = tableRow[2];
            double flywheelVelocity = tableRow[3];

            lowGoalHoodAngleMap.put(new InterpolatingDouble(distance), new InterpolatingDouble(hoodAngle));
            lowGoalFlywheelVelocityMap.put(new InterpolatingDouble(distance), new InterpolatingDouble(flywheelVelocity));
        }

        for (double[] tableRow : highGoalMap) {
            double distance = tableRow[1];
            double hoodAngle = tableRow[2];
            double flywheelVelocity = tableRow[3];

            highGoalHoodAngleMap.put(new InterpolatingDouble(distance), new InterpolatingDouble(hoodAngle));
            highGoalFlywheelVelocityMap.put(new InterpolatingDouble(distance), new InterpolatingDouble(flywheelVelocity));
        }

        // Angles in reference to fixed arm
        // 200:1 GR
        // Encoder counts = deg * (1 pivot arm rev / 360 deg) * (200 pivot motor rev / 1 pivot arm rev) * (2048 counts / 1 pivot motor rev)
        // Tolerance: 1 deg
        climberPivotTargets.put(LINING_UP_FOR_MID_BAR, new Target(11375, 75));
        climberPivotTargets.put(STOWED_ANGLE, new Target(0, 75)); // 0 deg
        climberPivotTargets.put(ANGLE_HOOK_THETA_FOR_MID_BAR, new Target(-2000, 75)); // -9.7 deg
        climberPivotTargets.put(PIVOT_BACK_TO_TRANSFER, new Target(13774, 75));
        climberPivotTargets.put(REACHING_FOR_HIGH_BAR_PIVOT_ANGLE, new Target(25824, 75)); // 23.4 deg
        climberPivotTargets.put(ANGLE_TO_HOOK_ONTO_HIGH_BAR, new Target(-3675, 75)); // 21.5 deg
        climberPivotTargets.put(ANGLE_TO_POSITION_FIXED_ARM_FOR_HIGH_BAR_TRANSFER, new Target(-2000, 75)); // -10.4 deg
        climberPivotTargets.put(FIXED_ARM_TO_HOOK_ONTO_HIGH_BAR_ANGLE, new Target(16000, 75)); // -1.0 deg
        climberPivotTargets.put(REACHING_FOR_TRAVERSAL_BAR_PIVOT_ANGLE, new Target(30265, 75)); // 26.6 deg
        climberPivotTargets.put(ANGLE_TO_HOOK_ONTO_TRAVERSAL_BAR, new Target(32995, 75)); // 20.0 deg

        // Lengths are relative to uppermost ring of outer arm
        // 36:1 GR
        // Output shaft: 0.5 inch diameter (may change if spool is added)
        // Encoder counts = inches * (1 output rev / 0.5*pi inches) * (36 extension motor rev / 1 output rev) * (2048 counts / 1 extension motor rev)
        // Tolerance: 0.1 in
        climberExtensionTargets.put(STOWED_HEIGHT, new Target(6500, 1500)); // 1 in
        climberExtensionTargets.put(MID_BAR_CLIMB_LINING_UP_TO_MID_BAR_LENGTH, new Target(384261, 1877)); // 21.467 in
        climberExtensionTargets.put(HIGH_BAR_CLIMB_LINING_UP_TO_MID_BAR_LENGTH, new Target(369664, 1877)); // 21.467 in
        climberExtensionTargets.put(PULL_UP_TO_HOOK_ONTO_MID_BAR_LENGTH, new Target(120000, 1877)); // 11.0 in
        climberExtensionTargets.put(LENGTH_TO_DISENGAGE_FROM_MID_BAR, new Target(40549, 1877)); // 3.0 in
        climberExtensionTargets.put(HOOKING_ONTO_HIGH_BAR_LENGTH, new Target(469500, 1877)); // 25 in
        climberExtensionTargets.put(PULLING_UP_TO_HIGH_BAR_TRANSFER_LENGTH, new Target(120000, 1877)); // 13.50 in
        climberExtensionTargets.put(LENGTH_TO_DISENGAGE_FROM_HIGH_BAR, new Target(40549, 1877)); // 3.0 in
        climberExtensionTargets.put(HOOKING_ONTO_TRAVERSAL_BAR_LENGTH, new Target(469867, 1877)); // 26.0 in
        climberExtensionTargets.put(LENGTH_TO_HANG_FROM_TRAVERSAL_BAR, new Target(199012, 1877)); // 11.6 in
    }//samit and armand is a kool kid

    @Override
    public Target getPivotTarget(Climber.ClimberPivotPosition target) {
        return climberPivotTargets.get(target);
    }

    @Override
    public Target getExtensionTarget(Climber.ClimberExtensionPosition target) {
        return climberExtensionTargets.get(target);
    }

}
