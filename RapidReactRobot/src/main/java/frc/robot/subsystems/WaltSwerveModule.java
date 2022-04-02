package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatusFrame;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.geometry.Translation2d;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Preferences;
import frc.lib.strykeforce.swerve.SwerveModule;
import frc.robot.util.UtilMethods;

import java.util.logging.Level;

import static frc.robot.RobotContainer.robotLogger;

public class WaltSwerveModule implements SubSubsystem, SwerveModule {

    final int k100msPerSecond = 10;

    private final CANSparkMax azimuthSparkMax;
    private final BaseTalon driveTalon;
    private final DutyCycle azimuthAbsoluteEncoderPWM;
    private final boolean isAzimuthAbsoluteEncoderInverted;
    private final double azimuthAbsoluteCountsPerRev;
    private final double driveMetersPerNU;
    private final double driveDeadbandMetersPerSecond;
    private final double driveMaximumMetersPerSecond;
    private final edu.wpi.first.math.geometry.Translation2d wheelLocationMeters;
    private final SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0.74425, 2.3973, 0.072907);

    private double previousEncDistance = 0;
    private Translation2d position;
    private Translation2d startingPosition;
    private Pose2d estimatedRobotPose = new Pose2d();

    private final PeriodicIO periodicIO = new PeriodicIO();
    private Rotation2d previousAngle = new Rotation2d();

    private DriveControlState driveControlState = DriveControlState.OPEN_LOOP;

    public static class PeriodicIO {
        // Outputs
        public double azimuthRelativeCountsDemand;
        public double driveDemand;
        public double driveFeedforward;

        // Inputs
        public boolean hasDriveControllerReset;
        public int azimuthAbsoluteFrequency;
        public double azimuthAbsoluteOutput;
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
        azimuthAbsoluteEncoderPWM = builder.azimuthAbsoluteEncoderPWM;
        isAzimuthAbsoluteEncoderInverted = builder.isAzimuthAbsoluteEncoderInverted;
        azimuthAbsoluteCountsPerRev = builder.azimuthAbsoluteCountsPerRev;
        driveMetersPerNU = builder.driveMetersPerNU;
        driveDeadbandMetersPerSecond = builder.driveDeadbandMetersPerSecond;
        driveMaximumMetersPerSecond = builder.driveMaximumMetersPerSecond;
        wheelLocationMeters = builder.wheelLocationMeters;

        previousEncDistance = 0;

        Translation2d startingPose = new Translation2d(builder.wheelLocationMeters.getX(),
                builder.wheelLocationMeters.getY());

        resetDriveEncoder();

        position = startingPose;
        this.startingPosition = startingPose;
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
        periodicIO.azimuthAbsoluteFrequency = azimuthAbsoluteEncoderPWM.getFrequency();
        periodicIO.azimuthAbsoluteOutput = azimuthAbsoluteEncoderPWM.getOutput();
        periodicIO.azimuthRelativeCounts = azimuthSparkMax.getEncoder().getPosition();
        periodicIO.drivePositionNU = driveTalon.getSelectedSensorPosition();
        periodicIO.driveVelocityNU = driveTalon.getSelectedSensorVelocity();
        periodicIO.driveClosedLoopErrorNU = driveTalon.getClosedLoopError();
    }

    @Override
    public synchronized void outputData() {
        if (periodicIO.hasDriveControllerReset) {
            configDriveStatusFrames();
        }

        azimuthSparkMax.getPIDController().setReference(periodicIO.azimuthRelativeCountsDemand, CANSparkMax.ControlType.kPosition);

        if (driveControlState == DriveControlState.OPEN_LOOP) {
            driveTalon.set(ControlMode.PercentOutput, periodicIO.driveDemand,
                    DemandType.ArbitraryFeedForward, periodicIO.driveFeedforward);
        } else if (driveControlState == DriveControlState.VELOCITY) {
            driveTalon.set(ControlMode.Velocity, periodicIO.driveDemand,
                    DemandType.ArbitraryFeedForward, periodicIO.driveFeedforward);
        }
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
            double azimuthSetpoint = (azimuthAbsoluteCounts - reference) / azimuthAbsoluteCountsPerRev;

            azimuthSparkMax.getEncoder().setPosition(azimuthSetpoint);

            periodicIO.azimuthRelativeCountsDemand = azimuthSetpoint;
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
        return getAzimuthRelativeEncoderCounts() - periodicIO.azimuthRelativeCountsDemand;
    }

    public double getDriveVelocityErrorNU() {
        return periodicIO.driveClosedLoopErrorNU;
    }

    public boolean isAzimuthAbsoluteEncoderValid() {
        return periodicIO.azimuthAbsoluteFrequency >= 208 && periodicIO.azimuthAbsoluteFrequency <= 280;
    }

    public int getAzimuthAbsoluteEncoderCounts() {
        double output = periodicIO.azimuthAbsoluteOutput;

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

    public double getAzimuthRelativeEncoderCounts() {
        return periodicIO.azimuthRelativeCounts;
    }

    @Override
    public Rotation2d getAzimuthRotation2d() {
        double azimuthCounts = getAzimuthRelativeEncoderCounts();
        double radians = 2.0 * Math.PI * azimuthCounts;
        return new Rotation2d(radians);
    }

    public Rotation2d getFieldCentricAngle(Rotation2d robotHeading) {
        Rotation2d angle = getAzimuthRotation2d();
        Rotation2d normalizedAngle = Rotation2d.fromDegrees(UtilMethods.restrictAngle(angle.getDegrees(),
                0, 360));

        return normalizedAngle.rotateBy(robotHeading);
    }

    @Override
    public void setAzimuthRotation2d(Rotation2d angle) {
        setAzimuthOptimizedState(new SwerveModuleState(0.0, angle));
    }

    @Override
    public void setAbsoluteAzimuthRotation2d(Rotation2d angle) {
        // set the azimuth wheel position
        double countsBefore = getAzimuthRelativeEncoderCounts();
        double countsFromAngle = angle.getRadians() / (2.0 * Math.PI);
        double countsDelta = Math.IEEEremainder(countsFromAngle - countsBefore, 1.0);
        periodicIO.azimuthRelativeCountsDemand = countsBefore + countsDelta;
    }

    private SwerveModuleState setAzimuthOptimizedState(SwerveModuleState desiredState) {
        // minimize change in heading by potentially reversing the drive direction
        Rotation2d currentAngle = getAzimuthRotation2d();
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);

        // set the azimuth wheel position
        double countsBefore = getAzimuthRelativeEncoderCounts();
        double countsFromAngle = optimizedState.angle.getRadians() / (2.0 * Math.PI);
        double countsDelta = Math.IEEEremainder(countsFromAngle - countsBefore, 1.0);
        periodicIO.azimuthRelativeCountsDemand = countsBefore + countsDelta;

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

    private void setDriveOpenLoopMetersPerSecond(double metersPerSecond) {
        if (driveControlState != DriveControlState.OPEN_LOOP) {
            robotLogger.log(Level.FINEST, "Switching swerve module index {0} to open loop", new Object[]{getWheelIndex()});
            driveControlState = DriveControlState.OPEN_LOOP;
        }

        periodicIO.driveDemand = metersPerSecond / driveMaximumMetersPerSecond;
        periodicIO.driveFeedforward = 0;
    }

    public void setDriveClosedLoopMetersPerSecond(double metersPerSecond) {
        if (driveControlState != DriveControlState.VELOCITY) {
            robotLogger.log(Level.FINEST, "Switching swerve module index {0} to velocity", new Object[]{getWheelIndex()});
            driveControlState = DriveControlState.VELOCITY;
        }

        periodicIO.driveDemand = metersPerSecond / driveMetersPerNU / k100msPerSecond;
        periodicIO.driveFeedforward = feedforward.calculate(metersPerSecond) / 12.0;
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

    public Translation2d getPosition(){
        return position;
    }

    public Pose2d getEstimatedRobotPose(){
        return estimatedRobotPose;
    }

    public synchronized void updatePose(Rotation2d robotHeading){
        double currentEncDistance = getDrivePositionMeters();
        double deltaEncDistance = (currentEncDistance - previousEncDistance);
        Rotation2d currentWheelAngle = getFieldCentricAngle(robotHeading);
        Translation2d deltaPosition = new Translation2d(currentWheelAngle.getCos()*deltaEncDistance,
                currentWheelAngle.getSin()*deltaEncDistance);


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

    public synchronized void resetPose(Pose2d robotPose){
        Translation2d modulePosition = robotPose.transformBy(Pose2d.fromTranslation(startingPosition)).getTranslation();
        position = modulePosition;
    }

    public synchronized void resetPose(){
        position = startingPosition;
    }

    public synchronized void resetLastEncoderReading(){
        previousEncDistance = getDrivePositionMeters();
    }

    private void configDriveStatusFrames() {
        driveTalon.setStatusFramePeriod(StatusFrame.Status_1_General, 10);
        driveTalon.setStatusFramePeriod(StatusFrame.Status_2_Feedback0, 20);
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
        private BaseTalon driveTalon;
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
