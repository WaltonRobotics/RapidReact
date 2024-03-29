package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.lib.strykeforce.swerve.SwerveModule;

import java.util.logging.Level;

import static frc.robot.Constants.DriverPreferences.kUseAzimuthDeadband;
import static frc.robot.RobotContainer.robotLogger;

public class WaltSwerveModule implements SubSubsystem, SwerveModule {

    final int k100msPerSecond = 10;

    private final CANSparkMax azimuthSparkMax;
    private final BaseTalon driveTalon;
    private final Encoder quadratureEncoder;
    private final DutyCycle azimuthAbsoluteEncoderPWM;
    private final boolean isAzimuthAbsoluteEncoderInverted;
    private final double azimuthAbsoluteCountsPerRev;
    private final double driveMetersPerNU;
    private final double driveDeadbandMetersPerSecond;
    private final double driveMaximumMetersPerSecond;
    private final edu.wpi.first.math.geometry.Translation2d wheelLocationMeters;
//    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.74425, 2.3973, 0.072907);

    private double previousEncDistance = 0;
    private Translation2d position;
    private final Translation2d startingPosition;
    private Pose2d estimatedRobotPose = new Pose2d();

    private final PeriodicIO periodicIO = new PeriodicIO();
    private Rotation2d previousAngle = new Rotation2d();
    private double azimuthQuadratureOffset = 0;
    private final PIDController azimuthController;

    private DriveControlState driveControlState = DriveControlState.OPEN_LOOP;

    public static class PeriodicIO {
        // Outputs
        public double azimuthCountsDemand;
        public double driveDemand;
//        public double driveFeedforward;

        // Inputs
        public boolean hasDriveControllerReset;
//        public int azimuthAbsoluteFrequency;
//        public double azimuthAbsoluteOutput;
        public double azimuthRelativeCounts;
        public double driveVelocityNU;
        public double drivePositionNU;
        public double driveClosedLoopErrorNU;
    }

    private enum DriveControlState {
        OPEN_LOOP, VELOCITY
    }

    public WaltSwerveModule(Builder builder) {
        azimuthSparkMax = builder.azimuthSparkMax;

        driveTalon = builder.driveTalon;
        quadratureEncoder = builder.quadratureEncoder;
        azimuthAbsoluteEncoderPWM = builder.azimuthAbsoluteEncoderPWM;
        isAzimuthAbsoluteEncoderInverted = builder.isAzimuthAbsoluteEncoderInverted;
        azimuthAbsoluteCountsPerRev = builder.azimuthAbsoluteCountsPerRev;
        driveMetersPerNU = builder.driveMetersPerNU;
        driveDeadbandMetersPerSecond = builder.driveDeadbandMetersPerSecond;
        driveMaximumMetersPerSecond = builder.driveMaximumMetersPerSecond;
        wheelLocationMeters = builder.wheelLocationMeters;

        PIDController referenceController = builder.azimuthController;

        azimuthController = new PIDController(referenceController.getP(),
                referenceController.getI(), referenceController.getD());

        azimuthController.setTolerance(1);
//        azimuthController.enableContinuousInput(0, azimuthAbsoluteCountsPerRev);

        previousEncDistance = 0;

        Translation2d startingPose = new Translation2d(builder.wheelLocationMeters.getX(),
                builder.wheelLocationMeters.getY());

        resetDriveEncoder();

        position = startingPose;
        this.startingPosition = startingPose;

//        SmartDashboard.putNumber("Wheel " + getWheelIndex() + " p", azimuthController.getP());
    }

    @Override
    public synchronized void zeroSensors() {
        zeroSensors(new Pose2d());
    }

    public synchronized void zeroSensors(Pose2d robotPose) {
        resetPose(robotPose);
        estimatedRobotPose = robotPose;
        previousEncDistance = getDrivePositionMeters();
    }

    @Override
    public synchronized void collectData() {
        periodicIO.hasDriveControllerReset = driveTalon.hasResetOccurred();
//        periodicIO.azimuthAbsoluteFrequency = azimuthAbsoluteEncoderPWM.getFrequency();
//        periodicIO.azimuthAbsoluteOutput = azimuthAbsoluteEncoderPWM.getOutput();
        periodicIO.azimuthRelativeCounts = quadratureEncoder.getDistance();
        periodicIO.drivePositionNU = driveTalon.getSelectedSensorPosition();
        periodicIO.driveVelocityNU = driveTalon.getSelectedSensorVelocity();
        periodicIO.driveClosedLoopErrorNU = driveTalon.getClosedLoopError();
    }

    @Override
    public synchronized void outputData() {
        if (periodicIO.hasDriveControllerReset) {
            configDriveStatusFrames();
        }

//        SmartDashboard.putNumber("Module " + getWheelIndex() + " absolute demand", periodicIO.azimuthCountsDemand);

//        azimuthSparkMax.getPIDController().setReference(periodicIO.relativeCountsDemand, CANSparkMax.ControlType.kPosition);

//        azimuthController.setP(SmartDashboard.getNumber("Wheel " + getWheelIndex() + " p", azimuthController.getP()));

        double output = azimuthController.calculate(periodicIO.azimuthRelativeCounts,
                periodicIO.azimuthCountsDemand);

        if (azimuthController.atSetpoint() && kUseAzimuthDeadband) {
            azimuthSparkMax.set(0);
        } else {
            azimuthSparkMax.set(output);
        }

        if (driveControlState == DriveControlState.OPEN_LOOP) {
            driveTalon.set(ControlMode.PercentOutput, periodicIO.driveDemand);
        } else if (driveControlState == DriveControlState.VELOCITY) {
            driveTalon.set(ControlMode.Velocity, periodicIO.driveDemand);
        }
    }

    public double getDriveVoltage() {
        return driveTalon.getBusVoltage();
    }

    @Override
    public void updateShuffleboard() {

    }

    @Override
    public double getMaxSpeedMetersPerSecond() {
        return driveMaximumMetersPerSecond;
    }

    @Override
    public edu.wpi.first.math.geometry.Translation2d getWheelLocationMeters() {
        return wheelLocationMeters;
    }

    @Override
    public SwerveModuleState getState() {
        double speedMetersPerSecond = getDriveMetersPerSecond();
        Rotation2d angle = getAzimuthRotation2d();
        return new SwerveModuleState(speedMetersPerSecond, angle);
    }

    @Override
    public void setDesiredState(SwerveModuleState desiredState, boolean isDriveOpenLoop) {
        assert desiredState.speedMetersPerSecond >= 0.0;

        if (desiredState.speedMetersPerSecond < driveDeadbandMetersPerSecond) {
            desiredState = new SwerveModuleState(0.0, previousAngle);
        }

        SwerveModuleState optimizedState = setAzimuthOptimizedState(desiredState);

        if (isDriveOpenLoop) {
            setDriveOpenLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
        } else {
            setDriveClosedLoopMetersPerSecond(optimizedState.speedMetersPerSecond);
        }
    }

    @Override
    public void resetDriveEncoder() {
        var errorCode = driveTalon.setSelectedSensorPosition(0);
        if (errorCode.value != 0) {
            robotLogger.log(Level.WARNING, "Talon error code while resetting encoder to 0: {0}", errorCode);
        }
    }

    @Override
    public void storeAzimuthZeroReference() {
        int index = getWheelIndex();
        int position = getAzimuthAbsoluteEncoderCounts();
        String key = String.format("SwerveDrive/wheel.%d", index);
        Preferences.setInt(key, position);
        robotLogger.log(Level.INFO, "azimuth {0}: saved zero = {1}", new Object[]{index, position});
    }

    @Override
    public void storeAzimuthZeroReference(int absoluteCounts) {
        int index = getWheelIndex();
        String key = String.format("SwerveDrive/wheel.%d", index);
        Preferences.setInt(key, absoluteCounts);
        robotLogger.log(Level.INFO, "azimuth {0}: saved zero = {1}", new Object[]{index, absoluteCounts});
    }

    @Override
    public void loadAndSetAzimuthZeroReference() {
        int index = getWheelIndex();
        String key = String.format("SwerveDrive/wheel.%d", index);
        int reference = Preferences.getInt(key, Integer.MIN_VALUE);
        if (reference == Integer.MIN_VALUE) {
            robotLogger.log(Level.SEVERE, "no saved azimuth zero reference for swerve module {0}", index);
        }
        robotLogger.log(Level.INFO, "swerve module {0}: loaded azimuth zero reference = {1}", new Object[]{index, reference});

        double azimuthAbsoluteCounts = getAzimuthAbsoluteEncoderCounts();

        if (isAzimuthAbsoluteEncoderValid()) {
//            double azimuthSetpoint = (azimuthAbsoluteCounts - reference) / azimuthAbsoluteCountsPerRev;
//
//            azimuthSparkMax.getEncoder().setPosition(azimuthSetpoint);

            quadratureEncoder.reset();

            azimuthQuadratureOffset = azimuthAbsoluteCounts - reference;

            azimuthController.reset();
            azimuthController.setSetpoint(0);

            periodicIO.azimuthCountsDemand = 0;
        } else {
            robotLogger.log(Level.SEVERE, "failed to zero swerve module {0} due to invalid absolute encoder data", index);
        }
    }

    public CANSparkMax getAzimuthSparkMax() {
        return azimuthSparkMax;
    }

    public BaseTalon getDriveTalon() {
        return driveTalon;
    }

    public double getAzimuthPositionErrorNU() {
        return azimuthController.getPositionError();
    }

    public double getDriveVelocityErrorNU() {
        return periodicIO.driveClosedLoopErrorNU;
    }

    public boolean isAzimuthAbsoluteEncoderValid() {
        double frequency = azimuthAbsoluteEncoderPWM.getFrequency();

        return frequency >= 208 && frequency <= 280;
    }

    public double getAzimuthRelativeEncoderCounts() {
        return periodicIO.azimuthRelativeCounts;
    }

    public int getAzimuthAbsoluteEncoderCounts() {
        double output = azimuthAbsoluteEncoderPWM.getOutput();

        if (!isAzimuthAbsoluteEncoderValid()) {
            robotLogger.log(Level.SEVERE, "Absolute encoder data not valid!");
        }

        int position = (int) (Math.round(output * 4098.0) - 1);

        if (position < 0) {
            position = 0;
        } else if (position > 4095) {
            position = 4095;
        }

        if (isAzimuthAbsoluteEncoderInverted) {
            return 4095 - position;
        }

        return position;
    }

    @Override
    public Rotation2d getAzimuthRotation2d() {
        double azimuthCounts = (periodicIO.azimuthRelativeCounts + azimuthQuadratureOffset)
                / azimuthAbsoluteCountsPerRev;
        double radians = 2.0 * Math.PI * azimuthCounts;
        return new Rotation2d(radians);
    }

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading) {
        Rotation2d normalizedAngle = getAzimuthRotation2d();

        return normalizedAngle.rotateBy(robotHeading);
    }

    @Override
    public void setAzimuthRotation2d(Rotation2d angle) {
        setAzimuthOptimizedState(new SwerveModuleState(0.0, angle));
    }

    @Override
    public void setAbsoluteAzimuthRotation2d(Rotation2d angle) {
        // set the azimuth wheel position
        double countsBefore = periodicIO.azimuthRelativeCounts + azimuthQuadratureOffset;
        double countsFromAngle =
                angle.getRadians() / (2.0 * Math.PI) * azimuthAbsoluteCountsPerRev;
        double countsDelta = Math.IEEEremainder(countsFromAngle - countsBefore, azimuthAbsoluteCountsPerRev);
        periodicIO.azimuthCountsDemand = periodicIO.azimuthRelativeCounts + countsDelta;
    }

    private SwerveModuleState setAzimuthOptimizedState(SwerveModuleState desiredState) {
        // minimize change in heading by potentially reversing the drive direction
        Rotation2d currentAngle = getAzimuthRotation2d();
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);

        // set the azimuth wheel position
        setAbsoluteAzimuthRotation2d(optimizedState.angle);

        // save previous angle for use if inside deadband in setDesiredState()
        previousAngle = optimizedState.angle;
        return optimizedState;
    }

    public double getDrivePositionMeters() {
        return periodicIO.drivePositionNU * driveMetersPerNU;
    }

    public double getDriveMetersPerSecond() {
        return periodicIO.driveVelocityNU * driveMetersPerNU * k100msPerSecond;
    }

    public void setDriveOpenLoopMetersPerSecond(double metersPerSecond) {
        if (driveControlState != DriveControlState.OPEN_LOOP) {
            robotLogger.log(Level.FINEST, "Switching swerve module index {0} to open loop", new Object[]{getWheelIndex()});
            driveControlState = DriveControlState.OPEN_LOOP;
        }

        periodicIO.driveDemand = metersPerSecond / driveMaximumMetersPerSecond;
//        periodicIO.driveFeedforward = 0;
    }

    public void setDriveClosedLoopMetersPerSecond(double metersPerSecond) {
        if (driveControlState != DriveControlState.VELOCITY) {
            robotLogger.log(Level.FINEST, "Switching swerve module index {0} to velocity", new Object[]{getWheelIndex()});
            driveControlState = DriveControlState.VELOCITY;
        }

        periodicIO.driveDemand = metersPerSecond / driveMetersPerNU / k100msPerSecond;
//        periodicIO.driveFeedforward = feedforward.calculate(metersPerSecond) / 12.0;
    }

    public void setDriveClosedLoopVelocityNU(double velocityNU) {
        if (driveControlState != DriveControlState.VELOCITY) {
            robotLogger.log(Level.FINEST, "Switching swerve module index {0} to velocity", new Object[]{getWheelIndex()});
            driveControlState = DriveControlState.VELOCITY;
        }

        periodicIO.driveDemand = velocityNU;
//        periodicIO.driveFeedforward = feedforward.calculate(metersPerSecond) / 12.0;
    }

    public void setBrakeNeutralMode() {
        azimuthSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveTalon.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoastNeutralMode() {
        azimuthSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
        driveTalon.setNeutralMode(NeutralMode.Coast);
    }

    public int getWheelIndex() {
        if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() > 0) {
            return 0;
        }
        if (wheelLocationMeters.getX() > 0 && wheelLocationMeters.getY() < 0) {
            return 1;
        }
        if (wheelLocationMeters.getX() < 0 && wheelLocationMeters.getY() > 0) {
            return 2;
        }
        return 3;
    }

    public Translation2d getPosition() {
        return position;
    }

    public Pose2d getEstimatedRobotPose() {
        return estimatedRobotPose;
    }

    public synchronized void updatePose(Rotation2d robotHeading) {
        double currentEncDistance = getDrivePositionMeters();
        double deltaEncDistance = (currentEncDistance - previousEncDistance);
        Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
        Translation2d deltaPosition = new Translation2d(currentWheelAngle.getCos() * deltaEncDistance,
                currentWheelAngle.getSin() * deltaEncDistance);


        deltaPosition = new Translation2d(deltaPosition.x(),
                deltaPosition.y());
        Translation2d updatedPosition = position.translateBy(deltaPosition);
        Pose2d staticWheelPose = new Pose2d(updatedPosition, new com.team254.lib.geometry.Rotation2d(
                robotHeading.getDegrees()));
        Pose2d robotPose = staticWheelPose.transformBy(Pose2d.fromTranslation(startingPosition).inverse());
        position = updatedPosition;
        estimatedRobotPose = robotPose;
        previousEncDistance = currentEncDistance;
    }

    public synchronized void resetPose(Pose2d robotPose) {
        Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(startingPosition)).getTranslation();
        position = modulePosition;
    }

    public synchronized void resetPose() {
        position = startingPosition;
    }

    public synchronized void resetLastEncoderReading() {
        previousEncDistance = getDrivePositionMeters();
    }

    private void configDriveStatusFrames() {
        driveTalon.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        driveTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 10);
        driveTalon.setStatusFramePeriod(StatusFrame.Status_4_AinTempVbat, 1000);
        driveTalon.setStatusFramePeriod(StatusFrame.Status_10_Targets, 1000);
        driveTalon.setStatusFramePeriod(StatusFrame.Status_12_Feedback1, 1000);
        driveTalon.setStatusFramePeriod(StatusFrame.Status_13_Base_PIDF0, 100);
        driveTalon.setStatusFramePeriod(StatusFrame.Status_14_Turn_PIDF1, 1000);
        driveTalon.setStatusFramePeriod(StatusFrame.Status_15_FirmwareApiStatus, 1000);
        driveTalon.setStatusFramePeriod(StatusFrame.Status_17_Targets1, 1000);
    }

    @Override
    public String toString() {
        return "TalonSwerveModule{" + getWheelIndex() + '}';
    }

    public static class Builder {

        public static final int kDefaultTalonSRXCountsPerRev = 4096;
        public static final int kDefaultTalonFXCountsPerRev = 2048;
        private final int azimuthAbsoluteCountsPerRev = kDefaultTalonSRXCountsPerRev;
        private CANSparkMax azimuthSparkMax;
        private PIDController azimuthController;
        private BaseTalon driveTalon;
        private Encoder quadratureEncoder;
        private DutyCycle azimuthAbsoluteEncoderPWM;
        private boolean isAzimuthAbsoluteEncoderInverted;
        private double driveMetersPerNU;
        private double driveDeadbandMetersPerSecond = -1.0;
        private double driveMaximumMetersPerSecond;
        private edu.wpi.first.math.geometry.Translation2d wheelLocationMeters;

        public Builder() {
        }

        public Builder azimuthSparkMax(CANSparkMax azimuthSparkMax) {
            this.azimuthSparkMax = azimuthSparkMax;
            return this;
        }

        public Builder azimuthController(PIDController controller) {
            this.azimuthController = controller;
            return this;
        }

        public Builder driveTalon(BaseTalon driveTalon) {
            this.driveTalon = driveTalon;
            if (driveTalon instanceof TalonFX) {
                return this;
            }

            if (driveTalon instanceof TalonSRX) {
                return this;
            }

            throw new IllegalArgumentException("expect drive talon is TalonFX or TalonSRX");
        }

        public Builder quadratureEncoder(Encoder encoder) {
            this.quadratureEncoder = encoder;
            return this;
        }

        public Builder azimuthAbsoluteEncoderPWM(DutyCycle encoderPWM) {
            azimuthAbsoluteEncoderPWM = encoderPWM;
            return this;
        }

        public Builder isAzimuthAbsoluteEncoderInverted(boolean isInverted) {
            isAzimuthAbsoluteEncoderInverted = isInverted;
            return this;
        }

        public Builder driveMetersPerNU(double metersPerNU) {
            driveMetersPerNU = metersPerNU;
            return this;
        }

        public Builder driveDeadbandMetersPerSecond(double metersPerSecond) {
            driveDeadbandMetersPerSecond = metersPerSecond;
            return this;
        }

        // we currently only support TalonSRX for azimuth
        //    public Builder azimuthEncoderCountsPerRevolution(int countsPerRev) {
        //      azimuthCountsPerRev = countsPerRev;
        //      return this;
        //    }

        public Builder driveMaximumMetersPerSecond(double metersPerSecond) {
            driveMaximumMetersPerSecond = metersPerSecond;
            return this;
        }

        public Builder wheelLocationMeters(edu.wpi.first.math.geometry.Translation2d locationMeters) {
            wheelLocationMeters = locationMeters;
            return this;
        }

        public WaltSwerveModule build() {
            if (driveDeadbandMetersPerSecond < 0) {
                driveDeadbandMetersPerSecond = 0.01 * driveMaximumMetersPerSecond;
            }
            var module = new WaltSwerveModule(this);
            validateWaltonSwerveModuleObject(module);
            return module;
        }

        private void validateWaltonSwerveModuleObject(WaltSwerveModule module) {
            if (module.azimuthSparkMax == null) {
                throw new IllegalArgumentException("azimuth spark max must be set.");
            }

            if (module.azimuthController == null) {
                throw new IllegalArgumentException("azimuth controller must be set.");
            }

            if (module.azimuthAbsoluteEncoderPWM == null) {
                throw new IllegalArgumentException("azimuth encoder must be set.");
            }

            if (module.driveMetersPerNU <= 0) {
                throw new IllegalArgumentException("drive meters per NU must be greater than zero.");
            }

            if (module.azimuthAbsoluteCountsPerRev <= 0) {
                throw new IllegalArgumentException(
                        "azimuth encoder counts per revolution must be greater than zero.");
            }

            if (module.driveMaximumMetersPerSecond <= 0) {
                throw new IllegalArgumentException("drive maximum speed must be greater than zero.");
            }
        }
    }

}
