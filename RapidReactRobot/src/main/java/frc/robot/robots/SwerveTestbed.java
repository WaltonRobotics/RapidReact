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

    // Shooter constants
    private final TalonFXConfiguration flywheelMasterTalonConfig = new TalonFXConfiguration();
    private final TalonFXConfiguration flywheelSlaveTalonConfig = new TalonFXConfiguration();


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

        drivetrainConfig = new DrivetrainConfig() {
            @Override
            public SmartMotionConstants[] getAzimuthControllerConfigs() {
                return azimuthControllerConfigs;
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
            public double getWheelDiameterInches() {
                return 3.0;
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
            public double getDriveGearRatio() {
                final double kDriveMotorOutputGear = 12;
                final double kDriveInputGear = 21;
                final double kBevelInputGear = 15;
                final double kBevelOutputGear = 45;

                return (kDriveMotorOutputGear / kDriveInputGear) * (kBevelInputGear / kBevelOutputGear);
            }

            @Override
            public Translation2d[] getWheelLocationMeters() {
                return wheelLocationMeters;
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
            public double getLeftIntakeVoltage() {
                return 0;
            }

            @Override
            public double getRightIntakeVoltage() {
                return 0;
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
            public double getTransportIntakeVoltage() {
                return 0;
            }

            @Override
            public double getFeedIntakeVoltage() {
                return 0;
            }

            @Override
            public double getTransportOuttakeVoltage() {
                return 0;
            }

            @Override
            public double getFeedOuttakeVoltage() {
                return 0;
            }

            @Override
            public double getTransportShootVoltage() {
                return 0;
            }

            @Override
            public double getFeedShootVoltage() {
                return 0;
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
            public MotorConfig getLeftAdjustableHoodServoConfig() {
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
            public MotorConfig getRightAdjustableHoodServoConfig() {
                return new MotorConfig() {
                    @Override
                    public int getChannelOrID() {
                        return 7;
                    }

                    @Override
                    public boolean isInverted() {
                        return false;
                    }
                };
            }

            @Override
            public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterMap() {
                return mShooterMap;
            }

            @Override
            public InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> getShooterMap2(){
                return mShooterMap2;
            }

            @Override
            public double getMountingHeightInches() {
                return 32.00;
            }

            @Override
            public double getMountingAngleDegrees() {
                return 29;
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
            public double getAbsoluteCountsToIntegratedCountsFactor() {
                final double kAbsoluteCountsPerRev = 1.0;

                return getIntegratedCountsPerRev() / kAbsoluteCountsPerRev;
            }

            @Override
            public double getIntegratedCountsPerRev() {
                final double kPivotGearRatio = 200;
                return 2048.0 * kPivotGearRatio;
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
                        return 0;
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
            public int getLeftClimberLockChannel() {
                return 2;
            }

            @Override
            public int getRightClimberLockChannel() {
                return 3;
            }

            @Override
            public int getClimberDiscBrakeForwardChannel() {
                return 4;
            }

            @Override
            public int getClimberDiscBrakeReverseChannel() {
                return 5;
            }
        };
    }

    @Override
    public void defineLimits() {
        climberPivotLimits.put(PIVOT_STOWED, new LimitPair(-1138, 1138));
        climberPivotLimits.put(PIVOT_FULL_ROM, new LimitPair(-17067, 45511));
        climberPivotLimits.put(PIVOT_PULL_UP_TO_MID_BAR, new LimitPair(-12174, -9898));
        climberPivotLimits.put(PIVOT_PULL_UP_TO_HIGH_BAR, new LimitPair(23324, 25600));
        climberPivotLimits.put(PIVOT_PULL_UP_TO_TRANSFER_HIGH_BAR, new LimitPair(-12971, -10695));
        climberPivotLimits.put(PIVOT_PULL_UP_TO_TRAVERSAL_BAR, new LimitPair(21618, 23894));

        climberExtensionLimits.put(STOWED, new LimitPair(-4694, 4694));
        climberExtensionLimits.put(EXTENSION_FULL_ROM, new LimitPair(-4694, 965347));
        climberExtensionLimits.put(MID_BAR_POSITION_FIXED_ARM, new LimitPair(464673, 474061));
        climberExtensionLimits.put(HIGH_BAR_TRANSFER_TO_FIXED_ARM, new LimitPair(582015, 591403));
    }

    @Override
    public void defineTargets() {
        // Angles in reference to fixed arm
        // 200:1 GR
        // Encoder counts = deg * (1 pivot arm rev / 360 deg) * (200 pivot motor rev / 1 pivot arm rev) * (2048 counts / 1 pivot motor rev)
        // Tolerance: 1 deg
        climberPivotTargets.put(STOWED_ANGLE, new Target(0, 1138)); // 0 deg
        climberPivotTargets.put(ANGLE_HOOK_THETA_FOR_MID_BAR, new Target(-11036, 1138)); // -9.7 deg
        climberPivotTargets.put(REACHING_FOR_HIGH_BAR_PIVOT_ANGLE, new Target(26624, 1138)); // 23.4 deg
        climberPivotTargets.put(ANGLE_TO_HOOK_ONTO_HIGH_BAR, new Target(24462, 1138)); // 21.5 deg
        climberPivotTargets.put(ANGLE_TO_POSITION_FIXED_ARM_FOR_HIGH_BAR_TRANSFER, new Target(-11833, 1138)); // -10.4 deg
        climberPivotTargets.put(FIXED_ARM_TO_HOOK_ONTO_HIGH_BAR_ANGLE, new Target(-1138, 1138)); // -1.0 deg
        climberPivotTargets.put(REACHING_FOR_TRAVERSAL_BAR_PIVOT_ANGLE, new Target(30265, 1138)); // 26.6 deg
        climberPivotTargets.put(ANGLE_TO_HOOK_ONTO_TRAVERSAL_BAR, new Target(22756, 1138)); // 20.0 deg

        // Lengths are relative to uppermost ring of outer arm
        // 36:1 GR
        // Output shaft: 0.5 inch diameter (may change if spool is added)
        // Encoder counts = inches * (1 output rev / 0.5*pi inches) * (36 extension motor rev / 1 output rev) * (2048 counts / 1 extension motor rev)
        // Tolerance: 0.1 in
        climberExtensionTargets.put(STOWED_HEIGHT, new Target(0, 4694)); // 1 in
        climberExtensionTargets.put(LINING_UP_TO_MID_BAR_LENGTH, new Target(960653, 4694)); // 21.467 in
        climberExtensionTargets.put(PULL_UP_TO_HOOK_ONTO_MID_BAR_LENGTH, new Target(469367, 4694)); // 11.0 in
        climberExtensionTargets.put(LENGTH_TO_DISENGAGE_FROM_MID_BAR, new Target(93873, 4694)); // 3.0 in
        climberExtensionTargets.put(HOOKING_ONTO_HIGH_BAR_LENGTH, new Target(1126481, 4694)); // 25 in
        climberExtensionTargets.put(PULLING_UP_TO_HIGH_BAR_TRANSFER_LENGTH, new Target(586709, 4694)); // 13.50 in
        climberExtensionTargets.put(LENGTH_TO_DISENGAGE_FROM_HIGH_BAR, new Target(93873, 4694)); // 3.0 in
        climberExtensionTargets.put(HOOKING_ONTO_TRAVERSAL_BAR_LENGTH, new Target(1173418, 4694)); // 26.0 in
        climberExtensionTargets.put(LENGTH_TO_HANG_FROM_TRAVERSAL_BAR, new Target(497529, 4694)); // 11.6 in
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
