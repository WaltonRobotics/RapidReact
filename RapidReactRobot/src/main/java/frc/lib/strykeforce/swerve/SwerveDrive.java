package frc.lib.strykeforce.swerve;

import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.interfaces.Gyro;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.subsystems.WaltSwerveModule;
import frc.robot.util.AccelerationLimiter;

import java.util.Arrays;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;

import static frc.robot.RobotContainer.godSubsystem;

/** Control a Third Coast swerve drive. */
public class SwerveDrive {

  private final SwerveModule[] swerveModules;
  private final SwerveDriveKinematics kinematics;
  private final SwerveDriveOdometry odometry;
  private final Gyro gyro;
  private final double maxSpeedMetersPerSecond;
  private final AccelerationLimiter xLimiter;
  private final AccelerationLimiter yLimiter;
  private final AccelerationLimiter omegaLimiter;

  private Rotation2d gyroOffset = new Rotation2d();
  private boolean hasGyroOffset = false;

  /**
   * Construct a swerve drive object. Along with a gyro, this takes in four configured swerve
   * modules, by convention in left front, right front, left rear, right rear order.
   *
   * @param gyro the gyro to use for field-oriented driving
   * @param swerveModules the swerve modules
   */
  public SwerveDrive(Gyro gyro, AccelerationLimiter xLimiter, AccelerationLimiter yLimiter,
                     AccelerationLimiter omegaLimiter, SwerveModule... swerveModules) {
    this.gyro = gyro;
    this.xLimiter = xLimiter;
    this.yLimiter = yLimiter;
    this.omegaLimiter = omegaLimiter;
    this.swerveModules = swerveModules;
    final List<Translation2d> locations =
        Arrays.stream(swerveModules)
            .map(SwerveModule::getWheelLocationMeters)
            .collect(Collectors.toList());

    Translation2d[] translation2ds =
        Arrays.stream(swerveModules)
            .map(SwerveModule::getWheelLocationMeters)
            .toArray(Translation2d[]::new);

    // verify all swerve modules are set to same max speed
    Set<Double> maxSpeeds =
        Arrays.stream(swerveModules)
            .map(SwerveModule::getMaxSpeedMetersPerSecond)
            .collect(Collectors.toSet());

    if (maxSpeeds.size() > 1) {
      throw new IllegalStateException("swerve modules must have same driveMaximumMetersPerSecond");
    }
    maxSpeedMetersPerSecond = swerveModules[0].getMaxSpeedMetersPerSecond();

    kinematics = new SwerveDriveKinematics(translation2ds);
    odometry = new SwerveDriveOdometry(kinematics, gyro.getRotation2d().rotateBy(gyroOffset));
  }

  /**
   * Construct a swerve drive object with a navX gyro. This takes in four configured swerve modules,
   * by convention in left front, right front, left rear, right rear order.
   *
   * @param swerveModules the swerve modules
   */
  public SwerveDrive(AccelerationLimiter xLimiter, AccelerationLimiter yLimiter,
                     AccelerationLimiter omegaLimiter, SwerveModule... swerveModules) {
    this(new AHRS(), xLimiter, yLimiter, omegaLimiter, swerveModules);
  }

  /**
   * Returns the kinematics object in use by this swerve drive.
   *
   * @return The kinematics object in use.
   */
  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  /**
   * Returns the position of the robot on the field.
   *
   * @return the pose of the robot (x and y ane in meters)
   */
  public Pose2d getPoseMeters() {
    return odometry.getPoseMeters();
  }

  /**
   * Returns the current gyro-measured heading of the robot. This will be affected by any gyro drift
   * that may have accumulated since last gyro recalibration. The angle is continuous, that is it
   * will continue from 360 to 361 degrees. This allows algorithms that wouldn't want to see a
   * discontinuity in the gyro output as it sweeps past from 360 to 0 on the second time around. The
   * angle is expected to increase as the gyro turns counterclockwise when looked at from the top.
   *
   * @return the Rotation2d of the robot relative to gyro zero
   */
  public Rotation2d getHeading() {
    return hasGyroOffset ? gyro.getRotation2d().rotateBy(gyroOffset) : gyro.getRotation2d();
  }

  /**
   * Returns the current gyro-measured heading of the robot. This will be affected by any gyro drift
   * that may have accumulated since last gyro recalibration. The angle is continuous, that is it
   * will continue from 360 to 361 degrees. This allows algorithms that wouldn't want to see a
   * discontinuity in the gyro output as it sweeps past from 360 to 0 on the second time around. The
   * angle is expected to increase as the gyro turns clockwise when looked at from the top.
   *
   * @return the current heading in degrees of the robot relative to gyro zero
   */
  double getGyroAngle() {
    // FIXME: does not have gyro offset
    return gyro.getAngle();
  }

  /**
   * Return the rate of rotation of the gyro. The rate is based on the most recent reading of the
   * gyro analog value. The rate is expected to be positive as the gyro turns clockwise when looked
   * at from the top.
   *
   * @return the current rate in degrees per second
   */
  public double getGyroRate() {
    return gyro.getRate();
  }

  /**
   * Get the current gyro offset applied to the IMU gyro angle during field oriented driving.
   *
   * @return the gyro offset
   */
  public Rotation2d getGyroOffset() {
    return gyroOffset;
  }

  /**
   * Set the current gyro offset applied to the IMU gyro angle during field oriented driving,
   * defaults to zero.
   *
   * @param gyroOffset the desired offset
   */
  public void setGyroOffset(Rotation2d gyroOffset) {
    if (this.gyroOffset.equals(gyroOffset)) {
      return;
    }
    this.gyroOffset = gyroOffset;
    hasGyroOffset = true;
  }

  /**
   * Get the configured swerve modules.
   *
   * @return array of swerve modules
   */
  public SwerveModule[] getSwerveModules() {
    return swerveModules;
  }

  /**
   * Resets the robot's position on the field. Any accumulated gyro drift will be noted and
   * accounted for in subsequent calls to {@link #getPoseMeters()}.
   *
   * @param pose The robot's actual position on the field.
   */
  public void resetOdometry(Pose2d pose) {
    odometry.resetPosition(pose, gyro.getRotation2d().rotateBy(gyroOffset));
  }

  /** Resets the gyro to a heading of zero. */
  public void resetGyro() {
    gyro.reset();
  }

  public void reloadAzimuthZeros() {
    for (int i = 0; i < 4; i++) {
      swerveModules[i].loadAndSetAzimuthZeroReference();
    }
  }

  /**
   * Update the swerve drive odometry state. Call this from the drive subsystem {@code periodic()}
   * method.
   */
  public void periodic() {
    odometry.update(
        hasGyroOffset ? gyro.getRotation2d().rotateBy(gyroOffset) : gyro.getRotation2d(),
        swerveModules[0].getState(),
        swerveModules[1].getState(),
        swerveModules[2].getState(),
        swerveModules[3].getState());
  }

  /**
   * Drive the robot with given x, y, and rotational velocities using open-loop velocity control.
   *
   * @param vxMetersPerSecond the desired x velocity component
   * @param vyMetersPerSecond the desired y velocity component
   * @param omegaRadiansPerSecond the desired rotational velocity component
   * @param isFieldOriented true if driving field-oriented
   */
  public void drive(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      boolean isFieldOriented) {
    SwerveModuleState[] swerveModuleStates =
        getSwerveModuleStates(
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, isFieldOriented);
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setDesiredState(swerveModuleStates[i], true);
    }
  }

  /**
   * Move the robot with given x, y, and rotational velocities using closed-loop velocity control.
   *
   * @param vxMetersPerSecond the desired x velocity component
   * @param vyMetersPerSecond the desired y velocity component
   * @param omegaRadiansPerSecond the desired rotational velocity component
   * @param isFieldOriented true if driving field-oriented
   */
  public void move(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      boolean isFieldOriented) {
    SwerveModuleState[] swerveModuleStates =
        getSwerveModuleStates(
            vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond, isFieldOriented);
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setDesiredState(swerveModuleStates[i], false);
    }
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return kinematics.toChassisSpeeds(swerveModules[0].getState(),
            swerveModules[1].getState(),
            swerveModules[2].getState(),
            swerveModules[3].getState());
  }

  private SwerveModuleState[] getSwerveModuleStates(
      double vxMetersPerSecond,
      double vyMetersPerSecond,
      double omegaRadiansPerSecond,
      boolean isFieldOriented) {
    ChassisSpeeds chassisSpeeds =
        isFieldOriented
            ? ChassisSpeeds.fromFieldRelativeSpeeds(
                vxMetersPerSecond,
                vyMetersPerSecond,
                omegaRadiansPerSecond,
                hasGyroOffset ? gyro.getRotation2d().rotateBy(gyroOffset) : gyro.getRotation2d())
            : new ChassisSpeeds(vxMetersPerSecond, vyMetersPerSecond, omegaRadiansPerSecond);

//    SmartDashboard.putNumber("Vy", chassisSpeeds.vyMetersPerSecond);

    if (!godSubsystem.isInAuton()) {
      chassisSpeeds.vxMetersPerSecond = xLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
      chassisSpeeds.vyMetersPerSecond = yLimiter.calculate(chassisSpeeds.vyMetersPerSecond);

      if (!godSubsystem.isAutoAligning()) {
        chassisSpeeds.omegaRadiansPerSecond = omegaLimiter.calculate(chassisSpeeds.omegaRadiansPerSecond);
      }
    }

    var swerveModuleStates = kinematics.toSwerveModuleStates(chassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, maxSpeedMetersPerSecond);
    return swerveModuleStates;
  }

  /**
   * Directly set the swerve modules to the specified states.
   *
   * @param desiredStates the desired swerve module states
   */
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, maxSpeedMetersPerSecond);
    for (int i = 0; i < 4; i++) {
      swerveModules[i].setDesiredState(desiredStates[i], true);
    }
  }
}
