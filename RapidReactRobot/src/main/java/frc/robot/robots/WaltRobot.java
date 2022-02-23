package frc.robot.robots;

import frc.robot.config.*;
import frc.robot.subsystems.Climber;
import frc.robot.util.interpolation.InterpolatingDouble;
import frc.robot.util.interpolation.InterpolatingTreeMap;
import frc.robot.util.regression.PolynomialRegression;

// Generic superclass for all Walton robots and their characteristics
public abstract class WaltRobot {

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
