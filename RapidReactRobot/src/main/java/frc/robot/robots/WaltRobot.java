package frc.robot.robots;

import frc.robot.config.*;

// Generic superclass for all Walton robots and their characteristics
public abstract class WaltRobot {

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

    public abstract DrivetrainConfig getDrivetrainConfig();
    public abstract IntakeConfig getIntakeConfig();
    public abstract ConveyorConfig getConveyorConfig();
    public abstract ShooterConfig getShooterConfig();
    public abstract ClimberConfig getClimberConfig();

}
