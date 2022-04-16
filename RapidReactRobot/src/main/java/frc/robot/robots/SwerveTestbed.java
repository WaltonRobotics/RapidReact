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

import static frc.robot.subsystems.Climber.ClimberExtensionLimits.*;
import static frc.robot.subsystems.Climber.ClimberExtensionPosition.*;
import static frc.robot.subsystems.Climber.ClimberPivotLimits.*;
import static frc.robot.subsystems.Climber.ClimberPivotPosition.*;

public class SwerveTestbed extends WaltRobot {

    // Drivetrain constants
    private final SmartMotionConstants[] azimuthControllerConfigs = new SmartMotionConstants[4];
    private final TalonFXConfiguration[] driveControllerConfigs = new TalonFXConfiguration[4];

    private final double kDistanceBetweenWheelsWidthWiseMeters =
            Units.inchesToMeters(15.0 + 1.0 / 16.0 + 1.761652 * 2.0); // 18.586 in

    private final double kDistanceBetweenWheelsLengthWiseMeters =
            Units.inchesToMeters(16.0 + 17.0 / 32.0 + 1.761652 * 2.0); // 20.055 in

    private final Translation2d[] wheelLocationMeters = new Translation2d[4];

    private final double kTranslationalP = 6.0;
    private final double kTranslationalD = kTranslationalP / 100.0;

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

    private final PIDController faceDirectionController = new PIDController(0.05, 0, 0.000);
    private final PIDController autoAlignController = new PIDController(0.05, 0.015, 0.000);
    private final ProfiledPIDController turnToAngleController = new ProfiledPIDController
            (0.05, 0.015, 0.000, new TrapezoidProfile.Constraints(
                    Math.toDegrees(kMaxOmega / 1.1), 360.0));

    // Shooter constants
    private final TalonFXConfiguration flywheelMasterTalonConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration flywheelSlaveTalonConfig = new TalonFXConfiguration();

    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> flywheelVelocityMap
            = new InterpolatingTreeMap<>();
    private final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> hoodAngleMap
            = new InterpolatingTreeMap<>();

    // Climber constants
    private final TalonFXConfiguration pivotControllerTalonConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration extensionControllerTalonConfig = new TalonFXConfiguration();

    private final HashMap<Climber.ClimberPivotLimits, LimitPair> climberPivotLimits = new HashMap<>(5);
    private final HashMap<Climber.ClimberPivotPosition, Target> climberPivotTargets = new HashMap<>(10);

    private final HashMap<Climber.ClimberExtensionLimits, LimitPair> climberExtensionLimits = new HashMap<>(3);
    private final HashMap<Climber.ClimberExtensionPosition, Target> climberExtensionTargets = new HashMap<>(7);

    public SwerveTestbed() {
        configAll();
    }

    @Override
    public void configDrivetrain() {
        for (int i = 0; i < 4; i++) {
            SmartMotionConstants azimuthConfig = new SmartMotionConstants() {
                private final PIDController velocityPID = new PIDController(0.003198, 0.00006396, 0.0);

                @Override
                public PIDController getVelocityPID() {
                    return velocityPID;
                }

                @Override
                public double getIZone() {
                    return 0.0;
                }

                @Override
                public double getFeedforward() {
                    return 0.00997776;
                }

                @Override
                public double getMinOutput() {
                    return -1;
                }

                @Override
                public double getMaxOutput() {
                    return 1;
                }

                @Override
                public double getMaxVelocity() {
                    return 120;
                }

                @Override
                public double getMinOutputVelocity() {
                    return 0;
                }

                @Override
                public double getMaxAccel() {
                    return 100;
                }

                @Override
                public double getAllowedClosedLoopError() {
                    return 0;
                }
            };

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

            azimuthControllerConfigs[i] = azimuthConfig;
            driveControllerConfigs[i] = driveConfig;
        }

        final double x = kDistanceBetweenWheelsLengthWiseMeters / 2.0; // front-back, was ROBOT_LENGTH
        final double y = kDistanceBetweenWheelsWidthWiseMeters / 2.0; // left-right, was ROBOT_WIDTH

        wheelLocationMeters[0] = new Translation2d(x, y); // left front
        wheelLocationMeters[1] = new Translation2d(x, -y); // right front
        wheelLocationMeters[2] = new Translation2d(-x, y); // left rear
        wheelLocationMeters[3] = new Translation2d(-x, -y); // right rear

        turnToAngleController.enableContinuousInput(-180.0, 180.0);
        turnToAngleController.setTolerance(1.5, 1.0);

        drivetrainConfig = new DrivetrainConfig() {
            @Override
            public PIDController[] getAzimuthPositionalPIDs() {
                return new PIDController[]{
                        new PIDController(1.0, 0.0, 0.0),
                        new PIDController(1.0, 0.0, 0.0),
                        new PIDController(1.0, 0.0, 0.0),
                        new PIDController(1.0, 0.0, 0.0),
                };
            }

            @Override
            public TalonFXConfiguration[] getDriveControllerConfigs() {
                return driveControllerConfigs;
            }

            @Override
            public RelativeEncoderConfig[] getAzimuthQuadratureConfigs() {
                return new RelativeEncoderConfig[] {
                        new RelativeEncoderConfig() {
                            @Override
                            public double getDistancePerPulse() {
                                return 1;
                            }

                            @Override
                            public int getChannelA() {
                                return 5;
                            }

                            @Override
                            public int getChannelB() {
                                return 6;
                            }

                            @Override
                            public boolean isInverted() {
                                return true;
                            }
                        },

                        new RelativeEncoderConfig() {
                            @Override
                            public double getDistancePerPulse() {
                                return 1;
                            }

                            @Override
                            public int getChannelA() {
                                return 7;
                            }

                            @Override
                            public int getChannelB() {
                                return 8;
                            }

                            @Override
                            public boolean isInverted() {
                                return true;
                            }
                        },

                        new RelativeEncoderConfig() {
                            @Override
                            public double getDistancePerPulse() {
                                return 1;
                            }

                            @Override
                            public int getChannelA() {
                                return 9;
                            }

                            @Override
                            public int getChannelB() {
                                return 10;
                            }

                            @Override
                            public boolean isInverted() {
                                return true;
                            }
                        },

                        new RelativeEncoderConfig() {
                            @Override
                            public double getDistancePerPulse() {
                                return 1;
                            }

                            @Override
                            public int getChannelA() {
                                return 11;
                            }

                            @Override
                            public int getChannelB() {
                                return 12;
                            }

                            @Override
                            public boolean isInverted() {
                                return true;
                            }
                        }
                };
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
                return new boolean[]{true, true, false, true};
            }

            @Override
            public boolean[] getAbsoluteEncoderInversions() {
                return new boolean[]{true, true, true, true};
            }

            @Override
            public double getRelativeEncoderRotationsPerTick() {
                return 1.0 / (5.33 * 12.0);
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
                return 1.0;
            }

            @Override
            public double getDriveMetersPerNU() {
                return (1.0 / 5.25) / 2048.0 * Math.PI * Units.inchesToMeters(3.0);
            }

            @Override
            public Translation2d[] getWheelLocationMeters() {
                return wheelLocationMeters;
            }

            @Override
            public AccelerationLimiter getXLimiter() {
                return null;
            }

            @Override
            public AccelerationLimiter getYLimiter() {
                return null;
            }

            @Override
            public AccelerationLimiter getOmegaLimiter() {
                return null;
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
                return 0.6;
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
                        return 0;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public MotorConfig getRightIntakeControllerConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 1;
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
                return 1;
            }

            @Override
            public double getLeftIntakePercentOutput() {
                return 0.8;
            }

            @Override
            public double getRightIntakePercentOutput() {
                return 0.8;
            }

            @Override
            public double getLeftOuttakePercentOutput() {
                return -0.8;
            }

            @Override
            public double getRightOuttakePercentOutput() {
                return -0.8;
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
                        return 2;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public MotorConfig getFeedControllerConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 3;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public double getTransportIntakePercentOutput() {
                return 0.65;
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
        flywheelMasterTalonConfig.voltageCompSaturation = 11.0;
        flywheelSlaveTalonConfig.voltageCompSaturation = 11.0;

        // Spinning up profile
        flywheelMasterTalonConfig.slot0.kF = 0.0001;
        flywheelMasterTalonConfig.slot0.kP = 0.0001;
        flywheelMasterTalonConfig.slot0.kI = 0.0001;
        flywheelMasterTalonConfig.slot0.kD = 0.0001;
        flywheelMasterTalonConfig.slot0.allowableClosedloopError = 0;
        flywheelMasterTalonConfig.slot0.integralZone = 100;
        flywheelMasterTalonConfig.slot0.maxIntegralAccumulator = 0;
        flywheelMasterTalonConfig.slot0.closedLoopPeakOutput = 1.0;

        // Shooting profile
        flywheelMasterTalonConfig.slot1.kF = 0.0001;
        flywheelMasterTalonConfig.slot1.kP = 0.0001;
        flywheelMasterTalonConfig.slot1.kI = 0.0001;
        flywheelMasterTalonConfig.slot1.kD = 0.0001;
        flywheelMasterTalonConfig.slot1.allowableClosedloopError = 0;
        flywheelMasterTalonConfig.slot1.integralZone = 100;
        flywheelMasterTalonConfig.slot1.maxIntegralAccumulator = 0;
        flywheelMasterTalonConfig.slot1.closedLoopPeakOutput = 1.0;

        shooterConfig = new ShooterConfig() {
            @Override
            public MotorConfig getFlywheelMasterControllerMotorConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 4;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public MotorConfig getFlywheelSlaveControllerMotorConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 5;
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
                        return 6;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public double getLimelightMountingHeightInches() {
                return 32.00;
            }

            @Override
            public double getLimelightMountingAngleDegrees() {
                return 29;
            }

            @Override
            public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getFlywheelVelocityMap(Shooter.AimTarget target) {
                return flywheelVelocityMap;
            }

            @Override
            public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getHoodAngleMap(Shooter.AimTarget target) {
                return hoodAngleMap;
            }
        };
    }

    @Override
    public void configClimber() {
        pivotControllerTalonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                true, 75, 80, 1);
        pivotControllerTalonConfig.voltageCompSaturation = 12.0;
        pivotControllerTalonConfig.forwardSoftLimitEnable = true;
        pivotControllerTalonConfig.reverseSoftLimitEnable = true;

        // Motion Magic slot
        pivotControllerTalonConfig.slot0.kF = 0.0001;
        pivotControllerTalonConfig.slot0.kP = 0.0001;
        pivotControllerTalonConfig.slot0.kI = 0;
        pivotControllerTalonConfig.slot0.kD = 0;
        pivotControllerTalonConfig.slot0.allowableClosedloopError = 0;
        pivotControllerTalonConfig.slot0.integralZone = 100;
        pivotControllerTalonConfig.slot0.maxIntegralAccumulator = 0;
        pivotControllerTalonConfig.slot0.closedLoopPeakOutput = 1.0;
        pivotControllerTalonConfig.motionCruiseVelocity = 100;
        pivotControllerTalonConfig.motionAcceleration = 100;
        pivotControllerTalonConfig.motionCurveStrength = 3;

        extensionControllerTalonConfig.supplyCurrLimit = new SupplyCurrentLimitConfiguration(
                true, 75, 80, 1);
        extensionControllerTalonConfig.voltageCompSaturation = 12.0;
        extensionControllerTalonConfig.forwardSoftLimitEnable = true;
        extensionControllerTalonConfig.reverseSoftLimitEnable = true;

        // Motion Magic slot
        extensionControllerTalonConfig.slot0.kF = 0.0001;
        extensionControllerTalonConfig.slot0.kP = 0.0001;
        extensionControllerTalonConfig.slot0.kI = 0.0001;
        extensionControllerTalonConfig.slot0.kD = 0.0001;
        extensionControllerTalonConfig.slot0.allowableClosedloopError = 0;
        extensionControllerTalonConfig.slot0.integralZone = 100;
        extensionControllerTalonConfig.slot0.maxIntegralAccumulator = 0;
        extensionControllerTalonConfig.slot0.closedLoopPeakOutput = 1.0;
        extensionControllerTalonConfig.motionCruiseVelocity = 100;
        extensionControllerTalonConfig.motionAcceleration = 100;
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
                return 0;
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
                        return 8;
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
                        return 9;
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
                        return 1;
                    }

                    @Override
                    public int getChannel() {
                        return 5;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public int getLeftExtensionLowerLimitChannel() {
                return 6;
            }

            @Override
            public int getRightExtensionLowerLimitChannel() {
                return 7;
            }

            @Override
            public int getClimberLockSolenoidChannel() {
                return 2;
            }

            @Override
            public int getHighBarArmsSolenoidChannel() {
                return 4;
            }

            @Override
            public double getManualPivotPercentOutputLimit() {
                return 0.3;
            }

            @Override
            public double getExtensionManualPercentOutputLimit() {
                return 0.3;
            }

            @Override
            public double getAbsoluteCountsToIntegratedCountsFactor() {
                return 0;
            }
        };
    }

    @Override
    public void defineLimits() {
        climberPivotLimits.put(PIVOT_STOWED, new LimitPair(956, 1006));
        climberPivotLimits.put(PIVOT_FULL_ROM, new LimitPair(969, 1065));

        climberExtensionLimits.put(STOWED, new LimitPair(5000, 8000));
        climberExtensionLimits.put(EXTENSION_FULL_ROM, new LimitPair(5000, 470081));
        climberExtensionLimits.put(MID_BAR_FINALIZE_CLIMB, new LimitPair(119000, 121000));
    }

    @Override
    public void defineTargets() {
        final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> sixtyDegreeMap = new InterpolatingTreeMap<>();

        sixtyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        sixtyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        sixtyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        sixtyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        sixtyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        sixtyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        sixtyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));

        final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> seventyDegreeMap = new InterpolatingTreeMap<>();

        seventyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        seventyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        seventyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        seventyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        seventyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        seventyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));
        seventyDegreeMap.put(new InterpolatingDouble(0.0), new InterpolatingDouble(11500.0));

        // Angles in reference to fixed arm
        // 200:1 GR
        // Encoder counts = deg * (1 pivot arm rev / 360 deg) * (200 pivot motor rev / 1 pivot arm rev) * (2048 counts / 1 pivot motor rev)
        // Tolerance: 1 deg
        climberPivotTargets.put(STOWED_ANGLE, new Target(0, 75)); // 0 deg

        // Lengths are relative to uppermost ring of outer arm
        // 36:1 GR
        // Output shaft: 0.5 inch diameter (may change if spool is added)
        // Encoder counts = inches * (1 output rev / 0.5*pi inches) * (36 extension motor rev / 1 output rev) * (2048 counts / 1 extension motor rev)
        // Tolerance: 0.1 in
        climberExtensionTargets.put(STOWED_HEIGHT, new Target(6500, 1500)); // 1 in
        climberExtensionTargets.put(CLOSE_IN_TO_ZERO_LENGTH, new Target(15000, 1877));
        climberExtensionTargets.put(PULL_UP_TO_HOOK_ONTO_MID_BAR_LENGTH, new Target(120000, 1877)); // 11.0 in
        climberExtensionTargets.put(MID_BAR_CLIMB_LINING_UP_TO_MID_BAR_LENGTH, new Target(384261, 1877)); // 21.467 in
        climberExtensionTargets.put(FINALIZE_HIGH_BAR_CLIMB_LENGTH, new Target(200000, 1877));
    }

    @Override
    public Target getPivotTarget(Climber.ClimberPivotPosition target) {
        return climberPivotTargets.get(target);
    }

    @Override
    public Target getExtensionTarget(Climber.ClimberExtensionPosition target) {
        return climberExtensionTargets.get(target);
    }

}
