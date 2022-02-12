package frc.robot.robots;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.config.*;

public class SwerveTestbed extends WaltRobot {

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
                    3.0,
                    0,
                    0,
                    new TrapezoidProfile.Constraints(kMaxOmega / 2.0, 3.14));

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

    @Override
    public DrivetrainConfig getDrivetrainConfig() {
        return new DrivetrainConfig() {
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
                return new int[] { 1, 2, 3, 4 };
            }

            @Override
            public int[] getDriveControllerIDs() {
                return new int[] { 11, 12, 13, 14 };
            }

            @Override
            public int[] getAbsoluteEncoderChannels() {
                return new int[] { 1, 2, 3, 4 };
            }

            @Override
            public boolean[] getAzimuthControllerInversions() {
                return new boolean[]{ true, true, true, true };
            }

            @Override
            public boolean[] getDriveControllerInversions() {
                return new boolean[]{ false, true, true, false };
            }

            @Override
            public boolean[] getAbsoluteEncoderInversions() {
                return new boolean[]{ true, true, true, true };
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
    public IntakeConfig getIntakeConfig() {
        return new IntakeConfig() {
            @Override
            public MotorConfig getLeftIntakeControllerConfig() {
                return null;
            }

            @Override
            public MotorConfig getRightIntakeControllerConfig() {
                return null;
            }
        };
    }

    @Override
    public ConveyorConfig getConveyorConfig() {
        return new ConveyorConfig() {
            @Override
            public MotorConfig getTransportControllerConfig() {
                return null;
            }

            @Override
            public MotorConfig getFeedControllerConfig() {
                return null;
            }
        };
    }

    @Override
    public ShooterConfig getShooterConfig() {
        return new ShooterConfig() {
            @Override
            public MotorConfig getFlywheelMasterControllerMotorConfig() {
                return null;
            }

            @Override
            public MotorConfig getFlywheelSlaveControllerMotorConfig() {
                return null;
            }

            @Override
            public TalonFXConfiguration getFlywheelMasterControllerTalonConfig() {
                return null;
            }

            @Override
            public TalonFXConfiguration getFlywheelSlaveControllerTalonConfig() {
                return null;
            }

            @Override
            public MotorConfig getLeftAdjustableHoodServoConfig() {
                return null;
            }

            @Override
            public MotorConfig getRightAdjustableHoodServoConfig() {
                return null;
            }
        };
    }

    @Override
    public ClimberConfig getClimberConfig() {
        return new ClimberConfig() {
            @Override
            public MotorConfig getPivotControllerMotorConfig() {
                return null;
            }

            @Override
            public MotorConfig getExtensionControllerMotorConfig() {
                return null;
            }

            @Override
            public TalonFXConfiguration getPivotControllerTalonConfig() {
                return null;
            }

            @Override
            public TalonFXConfiguration getExtensionControllerTalonConfig() {
                return null;
            }

            @Override
            public AbsoluteEncoderConfig getPivotAngleAbsoluteEncoderConfig() {
                return null;
            }

            @Override
            public int getLeftExtensionLowerLimitChannel() {
                return 0;
            }

            @Override
            public int getRightExtensionLowerLimitChannel() {
                return 0;
            }

            @Override
            public int getLeftClimberLockChannel() {
                return 0;
            }

            @Override
            public int getRightClimberLockChannel() {
                return 0;
            }

            @Override
            public int getClimberDiscBrakeForwardChannel() {
                return 0;
            }

            @Override
            public int getClimberDiscBrakeReverseChannel() {
                return 0;
            }
        };
    }

}
