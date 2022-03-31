package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.strykeforce.swerve.SwerveDrive;
import frc.robot.config.DrivetrainConfig;
import frc.robot.config.SmartMotionConstants;
import frc.robot.util.UtilMethods;

import static frc.robot.Constants.ContextFlags.kIsInCompetition;
import static frc.robot.Constants.DriverPreferences.kFaceDirectionToleranceDegrees;
import static frc.robot.Constants.FieldConstants.kCenterOfHubPose;
import static frc.robot.RobotContainer.currentRobot;
import static frc.robot.RobotContainer.godSubsystem;

public class Drivetrain extends SubsystemBase implements SubSubsystem {

    private final DrivetrainConfig config = currentRobot.getDrivetrainConfig();

    private final Field2d field = new Field2d();

    private final SwerveDrive swerveDrive;
    private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    private final WaltSwerveModule[] swerveModules;

    public Drivetrain() {
        var moduleBuilder =
                new WaltSwerveModule.Builder()
                        .driveGearRatio(config.getDriveGearRatio())
                        .wheelDiameterInches(config.getWheelDiameterInches())
                        .driveMaximumMetersPerSecond(config.getMaxSpeedMetersPerSecond());

        swerveModules = new WaltSwerveModule[4];

        for (int i = 0; i < 4; i++) {
            var azimuthSparkMax = new CANSparkMax(config.getAzimuthControllerIDs()[i], CANSparkMaxLowLevel.MotorType.kBrushless);
            azimuthSparkMax.restoreFactoryDefaults();
            azimuthSparkMax.enableVoltageCompensation(8.0);
            azimuthSparkMax.setSmartCurrentLimit(25);
            azimuthSparkMax.setOpenLoopRampRate(0.0);
            azimuthSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
            azimuthSparkMax.setInverted(config.getAzimuthControllerInversions()[i]);
            azimuthSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, false);
            azimuthSparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, false);

            // 1.0 reported by the azimuth relative encoder should correspond to 1 full rotation of the wheel
            RelativeEncoder azimuthRelativeEncoder = azimuthSparkMax.getEncoder();
            SparkMaxPIDController azimuthPID = azimuthSparkMax.getPIDController();

            azimuthRelativeEncoder.setPositionConversionFactor(config.getRelativeEncoderRotationsPerTick());
            azimuthRelativeEncoder.setVelocityConversionFactor(config.getRelativeEncoderRotationsPerTick());

            // Smart Motion Configuration
            azimuthPID.setP(config.getAzimuthPositionalPID().getP());
            azimuthPID.setI(config.getAzimuthPositionalPID().getI());
            azimuthPID.setD(config.getAzimuthPositionalPID().getD());
            azimuthPID.setIZone(0);
            azimuthPID.setFF(0);
            azimuthPID.setOutputRange(-1, 1);

            if (kIsInCompetition) {
                azimuthSparkMax.burnFlash();
            }

            TalonFXConfiguration driveConfig = config.getDriveControllerConfigs()[i];

            var driveTalon = new TalonFX(config.getDriveControllerIDs()[i]);
            driveTalon.configFactoryDefault(10);
            driveTalon.configAllSettings(driveConfig, 10);
            driveTalon.enableVoltageCompensation(true);
            driveTalon.setNeutralMode(NeutralMode.Coast);

            driveTalon.setInverted(config.getDriveControllerInversions()[i]);
            driveTalon.setSensorPhase(config.getDriveControllerInversions()[i]);

            DutyCycle encoderPWM = new DutyCycle(new DigitalInput(config.getAbsoluteEncoderChannels()[i]));

//            ProfiledPIDController controller = new ProfiledPIDController(
//                    /* 20.0 / 1023.0 */ 10.0 / 4096.0, 0.0, 0.0,
//                    new TrapezoidProfile.Constraints(800 * 10, 1000 * 10)
//            );
//
//            controller.setTolerance(30.0);
//            controller.enableContinuousInput(-90 * 4096.0, 90 * 4096.0);

            swerveModules[i] =
                    moduleBuilder
                            .azimuthSparkMax(azimuthSparkMax)
                            .driveTalon(driveTalon)
                            .azimuthAbsoluteEncoderPWM(encoderPWM)
                            .isAzimuthAbsoluteEncoderInverted(config.getAbsoluteEncoderInversions()[i])
                            .wheelLocationMeters(config.getWheelLocationMeters()[i])
                            .build();
        }

        swerveDrive = new SwerveDrive(ahrs, config.getXLimiter(), config.getYLimiter(), config.getOmegaLimiter(),
                swerveModules);

        SmartDashboard.putData("Field", field);
    }

    public void saveLeftFrontZero(int absoluteCounts) {
        swerveModules[0].storeAzimuthZeroReference(absoluteCounts);
    }

    public void saveRightFrontZero(int absoluteCounts) {
        swerveModules[1].storeAzimuthZeroReference(absoluteCounts);
    }

    public void saveLeftRearZero(int absoluteCounts) {
        swerveModules[2].storeAzimuthZeroReference(absoluteCounts);
    }

    public void saveRightRearZero(int absoluteCounts) {
        swerveModules[3].storeAzimuthZeroReference(absoluteCounts);
    }

    public Field2d getField() {
        return field;
    }

    /**
     * Returns the swerve drive kinematics object for use during trajectory configuration.
     *
     * @return the configured kinemetics object
     */
    public SwerveDriveKinematics getSwerveDriveKinematics() {
        return swerveDrive.getKinematics();
    }

    public void reloadAzimuthZeros() {
        swerveDrive.reloadAzimuthZeros();
    }

    public void zeroDriveEncoders() {
        swerveDrive.resetDriveEncoders();
    }

    public void zeroHeading() {
        setHeadingOffset(Rotation2d.fromDegrees(0.0));
        swerveDrive.resetGyro();
    }

    /**
     * Returns the configured swerve drive modules.
     */
    public WaltSwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    public void resetPose(Pose2d pose, PathPlannerTrajectory.PathPlannerState state) {
        setHeadingOffset(state.holonomicRotation.minus(Rotation2d.fromDegrees(180)));
        Pose2d holonomicPose = new Pose2d(pose.getX(), pose.getY(), state.holonomicRotation);
        swerveDrive.resetOdometry(holonomicPose);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return the pose of the robot (x and y ane in meters)
     */
    public Pose2d getPoseMeters() {
        return swerveDrive.getPoseMeters();
    }

    /**
     * Perform periodic swerve drive odometry update.
     */
    @Override
    public void periodic() {
        swerveDrive.periodic();

        field.setRobotPose(getPoseMeters());
//        LiveDashboardHelper.putRobotData(getPoseMeters());

//        SmartDashboard.putNumber("Robot pitch angle", ahrs.getPitch());
//        SmartDashboard.putNumber("Robot roll angle", ahrs.getRoll());
    }

    /**
     * Drive the robot with given x, y, and rotational velocities with open-loop velocity control.
     */
    public void drive(
            double vxMetersPerSecond, double vyMetersPerSecond, double omegaRadiansPerSecond) {
        swerveDrive.drive(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, true);
    }

    /**
     * Move the robot with given x, y, and rotational velocities with closed-loop velocity control.
     */
    public void move(
            double vxMetersPerSecond,
            double vyMetersPerSecond,
            double omegaRadiansPerSecond,
            boolean isFieldOriented) {
        swerveDrive.move(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, isFieldOriented);
    }

    public void faceDirection(double vx, double vy, Rotation2d theta, boolean isFieldRelative) {
        double currentHeading = UtilMethods.restrictAngle(getHeading().getDegrees(), -180, 180);
        double thetaTarget = UtilMethods.restrictAngle(theta.getDegrees(), -180, 180);
        double thetaError = thetaTarget - currentHeading;

        SmartDashboard.putNumber("Theta face direction error", thetaError);

        double output = config.getFaceDirectionController().calculate(currentHeading, thetaTarget);

        output = Math.signum(output) * UtilMethods.limitRange(
                Math.abs(output), config.getMinTurnOmega(), config.getMaxFaceDirectionOmega());

        if (Math.abs(thetaError) < kFaceDirectionToleranceDegrees) {
            output = 0;
        }

        move(vx, vy, output, isFieldRelative);
    }

    public void faceClosest(double vx, double vy, boolean isFieldRelative) {
        double currentHeading = UtilMethods.restrictAngle(getHeading().getDegrees(), 0, 360);

//        SmartDashboard.putNumber("Current face closest heading", currentHeading);

        if (currentHeading <= 90 || currentHeading >= 270) {
            faceDirection(vx, vy, Rotation2d.fromDegrees(0), isFieldRelative);
        } else {
            faceDirection(vx, vy, Rotation2d.fromDegrees(180), isFieldRelative);
        }
    }

    public void setModuleStates(SwerveModuleState state) {
        for (WaltSwerveModule module : swerveModules) {
            module.setDriveClosedLoopMetersPerSecond(state.speedMetersPerSecond);
            module.setAbsoluteAzimuthRotation2d(state.angle);
        }
    }

    public void setHeadingOffset(Rotation2d offsetRads) {
        swerveDrive.setGyroOffset(offsetRads);
    }

    public Rotation2d getHeading() {
        return swerveDrive.getHeading();
    }

    public Rotation2d getPitch() {
        return Rotation2d.fromDegrees(ahrs.getPitch());
    }

    public double getLeftFrontDriveTemp() {
        return swerveModules[0].getDriveTalon().getTemperature();
    }

    public double getLeftFrontTurnTemp() {
        return swerveModules[0].getAzimuthSparkMax().getMotorTemperature();
    }

    public double getRightFrontDriveTemp() {
        return swerveModules[1].getDriveTalon().getTemperature();
    }

    public double getRightFrontTurnTemp() {
        return swerveModules[1].getAzimuthSparkMax().getMotorTemperature();
    }

    public double getLeftBackDriveTemp() {
        return swerveModules[2].getDriveTalon().getTemperature();
    }

    public double getLeftBackTurnTemp() {
        return swerveModules[2].getAzimuthSparkMax().getMotorTemperature();
    }

    public double getRightBackDriveTemp() {
        return swerveModules[3].getDriveTalon().getTemperature();
    }

    public double getRightBackTurnTemp() {
        return swerveModules[3].getAzimuthSparkMax().getMotorTemperature();
    }

    public void setBrakeNeutralMode() {
        for (WaltSwerveModule module : swerveModules) {
            module.setBrakeNeutralMode();
        }
    }

    public void setCoastNeutralMode() {
        for (WaltSwerveModule module : swerveModules) {
            module.setCoastNeutralMode();
        }
    }

    public DrivetrainConfig getConfig() {
        return config;
    }

    public void xLockSwerveDrive() {
        if (!godSubsystem.isInAuton()) {
            swerveModules[0].setAzimuthRotation2d(Rotation2d.fromDegrees(45));
            swerveModules[1].setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
            swerveModules[2].setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
            swerveModules[3].setAzimuthRotation2d(Rotation2d.fromDegrees(45));
        }
    }

    public Rotation2d getEstimatedAngleToHub() {
        Pose2d targetRobotRelative = kCenterOfHubPose.relativeTo(getPoseMeters());

        return new Rotation2d(Math.atan2(targetRobotRelative.getY(), targetRobotRelative.getX()));
    }

    public WaltSwerveModule[] getModules() {
        return swerveModules;
    }

    @Override
    public void zeroSensors() {
        reloadAzimuthZeros();
        zeroDriveEncoders();
        zeroHeading();
    }

    @Override
    public void collectData() {
        for (WaltSwerveModule module : swerveModules) {
            module.collectData();
        }
    }

    @Override
    public void outputData() {
        for (WaltSwerveModule module : swerveModules) {
            module.outputData();
        }
    }

    @Override
    public void updateShuffleboard() {
        // Absolute encoder data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Absolute Counts", swerveModules[0].getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Absolute Counts", swerveModules[1].getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Absolute Counts", swerveModules[2].getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Absolute Counts", swerveModules[3].getAzimuthAbsoluteEncoderCounts());

        // Relative encoder data
//        SmartDashboard.putNumber("Left Front Relative Counts", swerveModules[0].getAzimuthRelativeEncoderCounts());
//        SmartDashboard.putNumber("Right Front Relative Counts", swerveModules[1].getAzimuthRelativeEncoderCounts());
//        SmartDashboard.putNumber("Left Rear Relative Counts", swerveModules[2].getAzimuthRelativeEncoderCounts());
//        SmartDashboard.putNumber("Right Rear Relative Counts", swerveModules[3].getAzimuthRelativeEncoderCounts());

        // Azimuth degree data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Angle Degrees", swerveModules[0].getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Angle Degrees", swerveModules[1].getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Angle Degrees", swerveModules[2].getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Angle Degrees", swerveModules[3].getAzimuthRotation2d().getDegrees());

        // Azimuth position error data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Position Error", swerveModules[0].getAzimuthPositionErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Position Error", swerveModules[1].getAzimuthPositionErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Position Error", swerveModules[2].getAzimuthPositionErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Position Error", swerveModules[3].getAzimuthPositionErrorNU());

        // Drive velocity data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Velocity Msec", swerveModules[0].getDriveMetersPerSecond());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Velocity Msec", swerveModules[1].getDriveMetersPerSecond());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Velocity Msec", swerveModules[2].getDriveMetersPerSecond());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Velocity Msec", swerveModules[3].getDriveMetersPerSecond());

        // Drive velocity error data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Velocity Error", swerveModules[0].getDriveVelocityErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Velocity Error", swerveModules[1].getDriveVelocityErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Velocity Error", swerveModules[2].getDriveVelocityErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Velocity Error", swerveModules[3].getDriveVelocityErrorNU());
    }

    public double getAngularVelocityDegreesPerSec() {
        return -ahrs.getRate();
    }

}
