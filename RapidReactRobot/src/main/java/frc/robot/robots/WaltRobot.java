package frc.robot.robots;

import frc.robot.config.*;

// Generic superclass for all Walton robots and their characteristics
public abstract class WaltRobot {

    protected final DrivetrainConfig drivetrainConfig = new DrivetrainConfig();
    protected final IntakeConfig intakeConfig = new IntakeConfig();
    protected final ConveyorConfig conveyorConfig = new ConveyorConfig();
    protected final ShooterConfig shooterConfig = new ShooterConfig();
    protected final ClimberConfig climberConfig = new ClimberConfig();

    protected WaltRobot() {
        configDrivetrain();
        configIntake();
        configConveyor();
        configShooter();
        configClimber();
    }

    public abstract void configDrivetrain();
    public abstract void configIntake();
    public abstract void configConveyor();
    public abstract void configShooter();
    public abstract void configClimber();

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
