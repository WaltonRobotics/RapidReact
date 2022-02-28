package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.DutyCycle;
import edu.wpi.first.wpilibj.Preferences;
import frc.lib.strykeforce.swerve.SwerveModule;

import java.util.logging.Level;

import static frc.robot.RobotContainer.robotLogger;

public class WaltSwerveModule implements SubSubsystem, SwerveModule {

    final int k100msPerSecond = 10;

    private final CANSparkMax azimuthSparkMax;
    private final BaseTalon driveTalon;
    private final DutyCycle azimuthAbsoluteEncoderPWM;
    private final boolean isAzimuthAbsoluteEncoderInverted;
    private final double azimuthAbsoluteCountsPerRev;
    private final double driveCountsPerRev;
    private final double driveGearRatio;
    private final double wheelCircumferenceMeters;
    private final double driveDeadbandMetersPerSecond;
    private final double driveMaximumMetersPerSecond;
    private final Translation2d wheelLocationMeters;

    private final PeriodicIO periodicIO = new PeriodicIO();
    private Rotation2d previousAngle = new Rotation2d();

    private DriveControlState driveControlState = DriveControlState.OPEN_LOOP;

    public static class PeriodicIO implements Sendable {
        // Outputs
        public double azimuthRelativeCountsDemand;
        public double driveDemand;

        // Inputs
        public int azimuthAbsoluteCounts;
        public double azimuthRelativeCounts;
        public double driveVelocityNU;
        public double driveClosedLoopErrorNU;

        @Override
        public void initSendable(SendableBuilder builder) {

        }
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
        driveCountsPerRev = builder.driveCountsPerRev;
        driveGearRatio = builder.driveGearRatio;
        wheelCircumferenceMeters = Math.PI * Units.inchesToMeters(builder.wheelDiameterInches);
        driveDeadbandMetersPerSecond = builder.driveDeadbandMetersPerSecond;
        driveMaximumMetersPerSecond = builder.driveMaximumMetersPerSecond;
        wheelLocationMeters = builder.wheelLocationMeters;
    }

    @Override
    public void zeroSensors() {
        loadAndSetAzimuthZeroReference();
        resetDriveEncoder();
    }

    @Override
    public void collectData() {
        periodicIO.azimuthAbsoluteCounts = getAzimuthAbsoluteEncoderMeasurement();
        periodicIO.azimuthRelativeCounts = azimuthSparkMax.getEncoder().getPosition();
        periodicIO.driveVelocityNU = driveTalon.getSelectedSensorVelocity();
        periodicIO.driveClosedLoopErrorNU = driveTalon.getClosedLoopError();
    }

    @Override
    public void outputData() {
        azimuthSparkMax.getPIDController().setReference(periodicIO.azimuthRelativeCountsDemand, CANSparkMax.ControlType.kSmartMotion);

        if (driveControlState == DriveControlState.OPEN_LOOP) {
            driveTalon.set(ControlMode.PercentOutput, periodicIO.driveDemand);
        } else if (driveControlState == DriveControlState.VELOCITY) {
            driveTalon.set(ControlMode.Velocity, periodicIO.driveDemand);
        }
    }

    @Override
    public Sendable getPeriodicIOSendable() {
        return periodicIO;
    }

    @Override
    public double getMaxSpeedMetersPerSecond() {
        return driveMaximumMetersPerSecond;
    }

    @Override
    public Translation2d getWheelLocationMeters() {
        return wheelLocationMeters;
    }

    public double getDriveCountsPerRev() {
        return driveCountsPerRev;
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
        previousAngle = desiredState.angle;

        Rotation2d currentAngle = getAzimuthRotation2d();
        SwerveModuleState optimizedState = SwerveModuleState.optimize(desiredState, currentAngle);
        setAzimuthRotation2d(optimizedState.angle);
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
            robotLogger.log(Level.WARNING, "no saved azimuth zero reference for swerve module {0}", index);
            throw new IllegalStateException();
        }
        robotLogger.log(Level.INFO, "swerve module {0}: loaded azimuth zero reference = {1}", new Object[]{index, reference});

        double azimuthAbsoluteCounts = getAzimuthAbsoluteEncoderCounts();

        double azimuthSetpoint = (azimuthAbsoluteCounts - reference) / azimuthAbsoluteCountsPerRev;

        azimuthSparkMax.getEncoder().setPosition(azimuthSetpoint);

        periodicIO.azimuthRelativeCountsDemand = azimuthSetpoint;
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

    public int getAzimuthAbsoluteEncoderCounts() {
        return periodicIO.azimuthAbsoluteCounts;
    }

    private int getAzimuthAbsoluteEncoderMeasurement() {
        boolean isAzimuthAbsoluteEncoderValid = false;
        int frequency;
        double output = 0;

        for (int i = 0; i < 10; i++) {
            frequency = azimuthAbsoluteEncoderPWM.getFrequency();
            output = azimuthAbsoluteEncoderPWM.getOutput();

            isAzimuthAbsoluteEncoderValid = frequency >= 208 && frequency <= 280;

            if (isAzimuthAbsoluteEncoderValid) {
                break;
            }
        }

        if (!isAzimuthAbsoluteEncoderValid) {
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

    public Rotation2d getAzimuthRotation2d() {
        double azimuthCounts = getAzimuthRelativeEncoderCounts();
        double radians = 2.0 * Math.PI * azimuthCounts;
        return new Rotation2d(radians);
    }

    public void setAzimuthRotation2d(Rotation2d angle) {
        double countsBefore = getAzimuthRelativeEncoderCounts();
        double countsFromAngle = angle.getRadians() / (2.0 * Math.PI);
        double countsDelta = Math.IEEEremainder(countsFromAngle - countsBefore, 1.0);
        periodicIO.azimuthRelativeCountsDemand = countsBefore + countsDelta;
    }

    public double getDriveMetersPerSecond() {
        double encoderCountsPer100ms = periodicIO.driveVelocityNU;
        double motorRotationsPer100ms = encoderCountsPer100ms / driveCountsPerRev;
        double wheelRotationsPer100ms = motorRotationsPer100ms * driveGearRatio;
        double metersPer100ms = wheelRotationsPer100ms * wheelCircumferenceMeters;
        return metersPer100ms * k100msPerSecond;
    }

    private void setDriveOpenLoopMetersPerSecond(double metersPerSecond) {
        if (driveControlState != DriveControlState.OPEN_LOOP) {
            robotLogger.log(Level.FINEST, "Switching swerve module index {0} to open loop", new Object[]{getWheelIndex()});
            driveControlState = DriveControlState.OPEN_LOOP;
        }

        periodicIO.driveDemand = metersPerSecond / driveMaximumMetersPerSecond;
    }

    public void setDriveClosedLoopMetersPerSecond(double metersPerSecond) {
        if (driveControlState != DriveControlState.VELOCITY) {
            robotLogger.log(Level.FINEST, "Switching swerve module index {0} to velocity", new Object[]{getWheelIndex()});
            driveControlState = DriveControlState.VELOCITY;
        }

        double wheelRotationsPerSecond = metersPerSecond / wheelCircumferenceMeters;
        double motorRotationsPerSecond = wheelRotationsPerSecond / driveGearRatio;
        double encoderCountsPerSecond = motorRotationsPerSecond * driveCountsPerRev;

        periodicIO.driveDemand = encoderCountsPerSecond / k100msPerSecond;
    }

    public void setBrakeNeutralMode() {
        azimuthSparkMax.setIdleMode(CANSparkMax.IdleMode.kBrake);
        driveTalon.setNeutralMode(NeutralMode.Brake);
    }

    public void setCoastNeutralMode() {
        azimuthSparkMax.setIdleMode(CANSparkMax.IdleMode.kCoast);
        driveTalon.setNeutralMode(NeutralMode.Coast);
    }

    private int getWheelIndex() {
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
        private double driveGearRatio;
        private double wheelDiameterInches;
        private int driveCountsPerRev = kDefaultTalonFXCountsPerRev;
        private double driveDeadbandMetersPerSecond = -1.0;
        private double driveMaximumMetersPerSecond;
        private Translation2d wheelLocationMeters;

        public Builder() {
        }

        public Builder azimuthSparkMax(CANSparkMax azimuthSparkMax) {
            this.azimuthSparkMax = azimuthSparkMax;
            return this;
        }

        public Builder driveTalon(BaseTalon driveTalon) {
            this.driveTalon = driveTalon;
            if (driveTalon instanceof TalonFX) {
                driveCountsPerRev = kDefaultTalonFXCountsPerRev;
                return this;
            }

            if (driveTalon instanceof TalonSRX) {
                driveCountsPerRev = kDefaultTalonSRXCountsPerRev;
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

        public Builder driveGearRatio(double ratio) {
            driveGearRatio = ratio;
            return this;
        }

        public Builder wheelDiameterInches(double diameterInches) {
            wheelDiameterInches = diameterInches;
            return this;
        }

        public Builder driveEncoderCountsPerRevolution(int countsPerRev) {
            driveCountsPerRev = countsPerRev;
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

        public Builder wheelLocationMeters(Translation2d locationMeters) {
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

            if (module.driveGearRatio <= 0) {
                throw new IllegalArgumentException("drive gear ratio must be greater than zero.");
            }

            if (module.azimuthAbsoluteCountsPerRev <= 0) {
                throw new IllegalArgumentException(
                        "azimuth encoder counts per revolution must be greater than zero.");
            }

            if (module.driveCountsPerRev <= 0) {
                throw new IllegalArgumentException(
                        "drive encoder counts per revolution must be greater than zero.");
            }

            if (module.wheelCircumferenceMeters <= 0) {
                throw new IllegalArgumentException("wheel diameter must be greater than zero.");
            }

            if (module.driveMaximumMetersPerSecond <= 0) {
                throw new IllegalArgumentException("drive maximum speed must be greater than zero.");
            }

            if (module.wheelLocationMeters == null) {
                throw new IllegalArgumentException("wheel location must be set.");
            }

            if (module.driveTalon instanceof TalonFX
                    && module.driveCountsPerRev != kDefaultTalonFXCountsPerRev) {
                robotLogger.log(Level.WARNING, "drive TalonFX counts per rev = {0}", module.driveCountsPerRev);
            }

            if (module.driveTalon instanceof TalonSRX
                    && module.driveCountsPerRev != kDefaultTalonSRXCountsPerRev) {
                robotLogger.log(Level.WARNING, "drive TalonSRX counts per rev = {0}", module.driveCountsPerRev);
            }
        }
    }

}
