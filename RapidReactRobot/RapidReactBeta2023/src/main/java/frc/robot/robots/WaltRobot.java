package frc.robot.robots;

import frc.robot.config.*;

// Generic superclass for all Walton robots and their characteristics
public abstract class WaltRobot {

    protected DrivetrainConfig drivetrainConfig;

    protected void configAll() {
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



}
