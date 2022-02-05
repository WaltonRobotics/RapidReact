package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.kauailabs.navx.frc.AHRS;
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
import frc.lib.strykeforce.swerve.SwerveModule;
import frc.robot.config.DrivetrainConfig;
import frc.robot.config.SmartMotionConstants;

import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.RobotContainer.currentRobot;

public class Drivetrain extends SubsystemBase implements SubSubsystem {

    private final DrivetrainConfig config = currentRobot.getDrivetrainConfig();

    private final Field2d field = new Field2d();

    private final SwerveDrive swerveDrive;
    private final AHRS ahrs = new AHRS(SPI.Port.kMXP);

    private final WaltSwerveModule[] swerveModules;

    public Drivetrain() {
        var moduleBuilder =
                new WaltSwerveModule.Builder()
                        .driveGearRatio(config.kDriveGearRatio)
                        .wheelDiameterInches(config.kWheelDiameterInches)
                        .driveMaximumMetersPerSecond(config.kMaxSpeedMetersPerSecond);

        swerveModules = new WaltSwerveModule[4];

        for (int i = 0; i < 4; i++) {
            var azimuthSparkMax = new CANSparkMax(i + 1, CANSparkMaxLowLevel.MotorType.kBrushless);
            azimuthSparkMax.restoreFactoryDefaults();
            azimuthSparkMax.enableVoltageCompensation(12.0);
            azimuthSparkMax.setSmartCurrentLimit(80);
            azimuthSparkMax.setOpenLoopRampRate(0.0);
            azimuthSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
            azimuthSparkMax.setInverted(config.azimuthControllerInversions[i]);

            RelativeEncoder azimuthRelativeEncoder = azimuthSparkMax.getEncoder();
            SparkMaxPIDController azimuthPID = azimuthSparkMax.getPIDController();

            azimuthRelativeEncoder.setPositionConversionFactor(config.kRelativeEncoderRotationsPerTick);
            azimuthRelativeEncoder.setVelocityConversionFactor(config.kRelativeEncoderRotationsPerTick);

            SmartMotionConstants azimuthConfig = config.azimuthControllerConfigs[i];

            // Smart Motion Configuration
            azimuthPID.setP(azimuthConfig.velocityPID.getP());
            azimuthPID.setI(azimuthConfig.velocityPID.getI());
            azimuthPID.setD(azimuthConfig.velocityPID.getD());
            azimuthPID.setIZone(azimuthConfig.iZone);
            azimuthPID.setFF(azimuthConfig.feedforward);
            azimuthPID.setOutputRange(azimuthConfig.minOutput, azimuthConfig.maxOutput);

            int smartMotionSlot = 0;
            azimuthPID.setSmartMotionMaxVelocity(azimuthConfig.maxVelocity, smartMotionSlot);
            azimuthPID.setSmartMotionMinOutputVelocity(azimuthConfig.minOutputVelocity, smartMotionSlot);
            azimuthPID.setSmartMotionMaxAccel(azimuthConfig.maxAccel, smartMotionSlot);
            azimuthPID.setSmartMotionAllowedClosedLoopError(azimuthConfig.allowedClosedLoopError, smartMotionSlot);

            TalonFXConfiguration driveConfig = config.driveControllerConfigs[i];

            var driveTalon = new TalonFX(i + 11);
            driveTalon.configFactoryDefault(config.kTalonConfigTimeout);
            driveTalon.configAllSettings(driveConfig, config.kTalonConfigTimeout);
            driveTalon.enableVoltageCompensation(true);
            driveTalon.setNeutralMode(NeutralMode.Brake);

            driveTalon.setInverted(config.driveControllerInversions[i]);
            driveTalon.setSensorPhase(config.driveControllerInversions[i]);

            DutyCycle encoderPWM = new DutyCycle(new DigitalInput(i));

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
                            .isAzimuthAbsoluteEncoderInverted(config.absoluteEncoderInversions[i])
                            .wheelLocationMeters(config.wheelLocationMeters[i])
                            .build();
        }

        swerveDrive = new SwerveDrive(ahrs, swerveModules);

        SmartDashboard.putData("Field", field);

        zeroSensors();
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
        swerveDrive.resetGyro();
    }

    /**
     * Returns the configured swerve drive modules.
     */
    public WaltSwerveModule[] getSwerveModules() {
        return swerveModules;
    }

    /**
     * Resets the robot's position on the field.
     *
     * @param pose the current pose
     */
    public void resetPose(Pose2d pose) {
        swerveDrive.resetOdometry(pose);
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return the pose of the robot (x and y ane in meters)
     */
    public Pose2d getPoseMeters() {
        return swerveDrive.getPoseMeters();
    }

    public void setVelocityAndRotation(double velocity, double rotationAngleDegrees) {
        for (WaltSwerveModule module : swerveModules) {
            module.setDriveClosedLoopMetersPerSecond(velocity);
            module.setAzimuthRotation2d(Rotation2d.fromDegrees(rotationAngleDegrees));
        }
    }

    /**
     * Perform periodic swerve drive odometry update.
     */
    @Override
    public void periodic() {
        swerveDrive.periodic();

        field.setRobotPose(getPoseMeters());
//        LiveDashboardHelper.putRobotData(getPoseMeters());

        SmartDashboard.putNumber(kDrivetrainLeftFrontAbsolutePositionKey, swerveModules[0].getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber(kDrivetrainRightFrontAbsolutePositionKey, swerveModules[1].getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber(kDrivetrainLeftRearAbsolutePositionKey, swerveModules[2].getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber(kDrivetrainRightRearAbsolutePositionKey, swerveModules[3].getAzimuthAbsoluteEncoderCounts());

        SmartDashboard.putNumber(kDrivetrainLeftFrontRelativePositionKey, swerveModules[0].getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber(kDrivetrainRightFrontRelativePositionKey, swerveModules[1].getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber(kDrivetrainLeftRearRelativePositionKey, swerveModules[2].getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber(kDrivetrainRightRearRelativePositionKey, swerveModules[3].getAzimuthRelativeEncoderCounts());

        SmartDashboard.putNumber(kDrivetrainLeftFrontAngleDegreesKey, swerveModules[0].getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber(kDrivetrainRightFrontAngleDegreesKey, swerveModules[1].getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber(kDrivetrainLeftRearAngleDegreesKey, swerveModules[2].getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber(kDrivetrainRightRearAngleDegreesKey, swerveModules[3].getAzimuthRotation2d().getDegrees());

        SmartDashboard.putNumber(kDrivetrainLeftFrontVelocityErrorKey, swerveModules[0].getDriveVelocityError());
        SmartDashboard.putNumber(kDrivetrainRightFrontVelocityErrorKey, swerveModules[1].getDriveVelocityError());
        SmartDashboard.putNumber(kDrivetrainLeftRearVelocityErrorKey, swerveModules[2].getDriveVelocityError());
        SmartDashboard.putNumber(kDrivetrainRightRearVelocityErrorKey, swerveModules[3].getDriveVelocityError());
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

    public void setModuleStates(SwerveModuleState state) {
        for (SwerveModule module : getSwerveModules()) {
            ((WaltSwerveModule) module).setDriveClosedLoopMetersPerSecond(state.speedMetersPerSecond);
            ((WaltSwerveModule) module).setAzimuthRotation2d(state.angle);
        }
    }

    public void setHeadingOffset(Rotation2d offsetRads) {
        swerveDrive.setGyroOffset(offsetRads);
    }

    public Rotation2d getHeading() {
        return swerveDrive.getHeading();
    }

    public DrivetrainConfig getConfig() {
        return config;
    }

    public void xLockSwerveDrive() {
        swerveModules[0].setAzimuthRotation2d(Rotation2d.fromDegrees(45));
        swerveModules[1].setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
        swerveModules[2].setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
        swerveModules[3].setAzimuthRotation2d(Rotation2d.fromDegrees(45));
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

}
