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
import com.team254.lib.geometry.Translation2d;
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
import frc.robot.commands.auton.LiveDashboardHelper;
import frc.robot.commands.auton.LiveDashboardTable;
import frc.robot.config.DrivetrainConfig;
import frc.robot.config.SmartMotionConstants;
import frc.robot.util.UtilMethods;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;

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

    private final ArrayList<WaltSwerveModule> swerveModules = new ArrayList<>();

    // Odometry
    private com.team254.lib.geometry.Pose2d pose = new com.team254.lib.geometry.Pose2d();
    double distanceTraveled;

    public Drivetrain() {
        var moduleBuilder =
                new WaltSwerveModule.Builder()
                        .driveGearRatio(config.getDriveGearRatio())
                        .wheelDiameterInches(config.getWheelDiameterInches())
                        .driveMaximumMetersPerSecond(config.getMaxSpeedMetersPerSecond());

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

            swerveModules.add(moduleBuilder
                            .azimuthSparkMax(azimuthSparkMax)
                            .driveTalon(driveTalon)
                            .azimuthAbsoluteEncoderPWM(encoderPWM)
                            .isAzimuthAbsoluteEncoderInverted(config.getAbsoluteEncoderInversions()[i])
                            .wheelLocationMeters(config.getWheelLocationMeters()[i])
                            .build());
        }

        swerveDrive = new SwerveDrive(ahrs, config.getXLimiter(), config.getYLimiter(), config.getOmegaLimiter(),
                (WaltSwerveModule[])swerveModules.toArray());

        SmartDashboard.putData("Field", field);

        zeroSensors();

        resetPose(kCenterOfHubPose, new PathPlannerTrajectory.PathPlannerState());

//        LiveDashboardTable.getInstance().setFollowingPath(true);
    }

    public void saveLeftFrontZero(int absoluteCounts) {
        swerveModules.get(0).storeAzimuthZeroReference(absoluteCounts);
    }

    public void saveRightFrontZero(int absoluteCounts) {
        swerveModules.get(1).storeAzimuthZeroReference(absoluteCounts);
    }

    public void saveLeftRearZero(int absoluteCounts) {
        swerveModules.get(2).storeAzimuthZeroReference(absoluteCounts);
    }

    public void saveRightRearZero(int absoluteCounts) {
        swerveModules.get(3).storeAzimuthZeroReference(absoluteCounts);
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

    public void zeroHeading() {
        setHeadingOffset(Rotation2d.fromDegrees(0.0));
        swerveDrive.resetGyro();
    }

    /**
     * Returns the configured swerve drive modules.
     */
    public ArrayList<WaltSwerveModule> getSwerveModules() {
        return swerveModules;
    }

    public void resetPose(Pose2d robotPose, PathPlannerTrajectory.PathPlannerState state) {
        zeroHeading();
        setHeadingOffset(state.holonomicRotation);

        com.team254.lib.geometry.Pose2d startingPose = new com.team254.lib.geometry.Pose2d(robotPose.getX(),
                robotPose.getY(), new com.team254.lib.geometry.Rotation2d(state.holonomicRotation.getDegrees()));

        swerveModules.forEach(m -> m.zeroSensors(startingPose));

        pose = startingPose;
        distanceTraveled = 0;
    }

    /**
     * Returns the position of the robot on the field.
     *
     * @return the pose of the robot (x and y ane in meters)
     */
    public Pose2d getPoseMeters() {
        return new Pose2d(pose.getTranslation().x(), pose.getTranslation().y(),
                new Rotation2d(pose.getRotation().getRadians()));
    }

    /** Playing around with different methods of odometry. This will require the use of all four modules, however. */
    public synchronized void alternatePoseUpdate() {
        double x = 0.0;
        double y = 0.0;

        double[][] distances = new double[4][2];

        for(WaltSwerveModule m : swerveModules) {
            m.updatePose(getHeading());
            double distance = m.getEstimatedRobotPose().getTranslation().distance(pose.getTranslation());
            int moduleID = m.getWheelIndex();
            distances[moduleID][0] = moduleID;
            distances[moduleID][1] = distance;
        }

        Arrays.sort(distances, Comparator.comparingDouble(a -> a[1]));

        List<WaltSwerveModule> modulesToUse = new ArrayList<>();

        double firstDifference = distances[1][1] - distances[0][1];
        double secondDifference = distances[2][1] - distances[1][1];
        double thirdDifference = distances[3][1] - distances[2][1];

        if(secondDifference > (1.5 * firstDifference)){
            modulesToUse.add(swerveModules.get((int)distances[0][0]));
            modulesToUse.add(swerveModules.get((int)distances[1][0]));
        }else if(thirdDifference > (1.5 * firstDifference)){
            modulesToUse.add(swerveModules.get((int)distances[0][0]));
            modulesToUse.add(swerveModules.get((int)distances[1][0]));
            modulesToUse.add(swerveModules.get((int)distances[2][0]));
        }else{
            modulesToUse.add(swerveModules.get((int)distances[0][0]));
            modulesToUse.add(swerveModules.get((int)distances[1][0]));
            modulesToUse.add(swerveModules.get((int)distances[2][0]));
            modulesToUse.add(swerveModules.get((int)distances[3][0]));
        }

        SmartDashboard.putNumber("Modules Used", modulesToUse.size());

        for(WaltSwerveModule m : modulesToUse){
            x += m.getEstimatedRobotPose().getTranslation().x();
            y += m.getEstimatedRobotPose().getTranslation().y();
        }

        com.team254.lib.geometry.Pose2d updatedPose;
        updatedPose = new com.team254.lib.geometry.Pose2d(new Translation2d(x / modulesToUse.size(), y / modulesToUse.size()),
                new com.team254.lib.geometry.Rotation2d(getHeading().getDegrees()));

        double deltaPos = updatedPose.getTranslation().distance(pose.getTranslation());
        distanceTraveled += deltaPos;
        pose = updatedPose;

        swerveModules.forEach(m -> m.resetPose(pose));
    }

    /**
     * Perform periodic swerve drive odometry update.
     */
    @Override
    public void periodic() {
//        swerveDrive.periodic();

//        SmartDashboard.putNumber("Robot pitch angle", ahrs.getPitch());
//        SmartDashboard.putNumber("Robot roll angle", ahrs.getRoll());
//        field.setRobotPose(getPoseMeters());

        alternatePoseUpdate();

        LiveDashboardHelper.putRobotData(getPoseMeters());
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
        return swerveModules.get(0).getDriveTalon().getTemperature();
    }

    public double getLeftFrontTurnTemp() {
        return swerveModules.get(0).getAzimuthSparkMax().getMotorTemperature();
    }

    public double getRightFrontDriveTemp() {
        return swerveModules.get(1).getDriveTalon().getTemperature();
    }

    public double getRightFrontTurnTemp() {
        return swerveModules.get(1).getAzimuthSparkMax().getMotorTemperature();
    }

    public double getLeftBackDriveTemp() {
        return swerveModules.get(2).getDriveTalon().getTemperature();
    }

    public double getLeftBackTurnTemp() {
        return swerveModules.get(2).getAzimuthSparkMax().getMotorTemperature();
    }

    public double getRightBackDriveTemp() {
        return swerveModules.get(3).getDriveTalon().getTemperature();
    }

    public double getRightBackTurnTemp() {
        return swerveModules.get(3).getAzimuthSparkMax().getMotorTemperature();
    }

    public void setBrakeNeutralMode() {
        swerveModules.forEach(WaltSwerveModule::setBrakeNeutralMode);
    }

    public void setCoastNeutralMode() {
        swerveModules.forEach(WaltSwerveModule::setCoastNeutralMode);
    }

    public DrivetrainConfig getConfig() {
        return config;
    }

    public void xLockSwerveDrive() {
        if (!godSubsystem.isInAuton()) {
            swerveModules.get(0).setAzimuthRotation2d(Rotation2d.fromDegrees(45));
            swerveModules.get(1).setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
            swerveModules.get(2).setAzimuthRotation2d(Rotation2d.fromDegrees(-45));
            swerveModules.get(3).setAzimuthRotation2d(Rotation2d.fromDegrees(45));
        }
    }

    public Rotation2d getEstimatedAngleToHub() {
        Pose2d targetRobotRelative = kCenterOfHubPose.relativeTo(getPoseMeters());

        return new Rotation2d(Math.atan2(targetRobotRelative.getY(), targetRobotRelative.getX()));
    }

    @Override
    public synchronized void zeroSensors() {
        reloadAzimuthZeros();
        zeroHeading();
    }

    @Override
    public synchronized void collectData() {
        swerveModules.forEach(WaltSwerveModule::collectData);
    }

    @Override
    public synchronized void outputData() {
        swerveModules.forEach(WaltSwerveModule::outputData);
    }

    @Override
    public void updateShuffleboard() {
        // Absolute encoder data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Absolute Counts", swerveModules.get(0).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Absolute Counts", swerveModules.get(1).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Absolute Counts", swerveModules.get(2).getAzimuthAbsoluteEncoderCounts());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Absolute Counts", swerveModules.get(3).getAzimuthAbsoluteEncoderCounts());

        // Relative encoder data
        SmartDashboard.putNumber("Left Front Relative Counts", swerveModules.get(0).getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber("Right Front Relative Counts", swerveModules.get(1).getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber("Left Rear Relative Counts", swerveModules.get(2).getAzimuthRelativeEncoderCounts());
        SmartDashboard.putNumber("Right Rear Relative Counts", swerveModules.get(3).getAzimuthRelativeEncoderCounts());

        // Azimuth degree data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Angle Degrees", swerveModules.get(0).getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Angle Degrees", swerveModules.get(1).getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Angle Degrees", swerveModules.get(2).getAzimuthRotation2d().getDegrees());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Angle Degrees", swerveModules.get(3).getAzimuthRotation2d().getDegrees());

        // Azimuth position error data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Angle Error", swerveModules.get(0).getAzimuthPositionErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Angle Error", swerveModules.get(1).getAzimuthPositionErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Angle Error", swerveModules.get(2).getAzimuthPositionErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Angle Error", swerveModules.get(3).getAzimuthPositionErrorNU());

        // Drive velocity data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Position Meters", swerveModules.get(0).getDrivePositionMeters());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Position Meters", swerveModules.get(1).getDrivePositionMeters());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Position Meters", swerveModules.get(2).getDrivePositionMeters());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Position Meters", swerveModules.get(3).getDrivePositionMeters());

        // Drive velocity error data
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Front Velocity Error", swerveModules.get(0).getDriveVelocityErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Front Velocity Error", swerveModules.get(1).getDriveVelocityErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Left Rear Velocity Error", swerveModules.get(2).getDriveVelocityErrorNU());
        SmartDashboard.putNumber("Drivetrain/Periodic IO/Right Rear Velocity Error", swerveModules.get(3).getDriveVelocityErrorNU());
    }

    public double getAngularVelocityDegreesPerSec() {
        return -ahrs.getRate();
    }

}
