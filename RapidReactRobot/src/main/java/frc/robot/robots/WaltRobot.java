package frc.robot.robots;

import frc.robot.config.*;
import frc.robot.subsystems.Climber;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.util.interpolation.InterpolatingTreeMap;
import frc.robot.util.regression.PolynomialRegression;

// Generic superclass for all Walton robots and their characteristics
public abstract class WaltRobot {

    public WaltRobot(double[][] distanceToVelocityTable){
        mShooterMap = new InterpolatingTreeMap<>();

        for (double[] pair : distanceToVelocityTable) {
            mShooterMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        mShooterMap2 = null;
    }

    public WaltRobot(double[][]distanceToVelocityTableHoodOne, double[][]distanceToVelocityTableHoodTwo){
        mShooterMap = new InterpolatingTreeMap<>();
        mShooterMap2 = new InterpolatingTreeMap<>();
        for (double[] pair : distanceToVelocityTableHoodOne) {
            mShooterMap.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
        for (double[] pair : distanceToVelocityTableHoodTwo) {
            mShooterMap2.put(new InterpolatingDouble(pair[0]), new InterpolatingDouble(pair[1]));
        }
    }

    public WaltRobot() {
        mShooterMap = null;
        mShooterMap2 = null;
    }

    protected final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterMap;
    protected final InterpolatingTreeMap<InterpolatingDouble, InterpolatingDouble> mShooterMap2;
    protected DrivetrainConfig drivetrainConfig;
    protected IntakeConfig intakeConfig;
    protected ConveyorConfig conveyorConfig;
    protected ShooterConfig shooterConfig;
    protected ClimberConfig climberConfig;

    protected void configAll() {
        configDrivetrain();
        configIntake();
        configConveyor();
        configShooter();
        configClimber();

        defineLimits();
        defineTargets();
    }

    public abstract void configDrivetrain();

    public abstract void configIntake();

    public abstract void configConveyor();

    public abstract void configShooter();

    public abstract void configClimber();

    public abstract void defineLimits();

    public abstract void defineTargets();

    public abstract Target getPivotTarget(Climber.ClimberPivotPosition target);

    public abstract Target getExtensionTarget(Climber.ClimberExtensionPosition target);

    public DrivetrainConfig getDrivetrainConfig() {
        return drivetrainConfig;
    }

    public IntakeConfig getIntakeConfig() {
        return intakeConfig;
    }

    public ConveyorConfig getConveyorConfig() {
        return conveyorConfig;
    }

    public ShooterConfig getShooterConfig() {
        return shooterConfig;
    }

    public ClimberConfig getClimberConfig() {
        return climberConfig;
    }

}
