package frc.robot.robots;

import frc.robot.config.*;
import frc.robot.subsystems.Climber;

public class CompRapidReact extends WaltRobot {
    //Dummy table values
    private static final double[][] mShooterDistanceToVelocityTableHoodOne = {
            {9.49, 12500},
            {10.32, 11975},
            {10.91, 11975},
            {12.03, 11475},
            {16.33, 11000},
            {25.85, 11075},
    };
    private static final double[][] mShooterDistanceToVelocityTableHoodTwo = {
            {9.49, 12500},
            {10.32, 11975},
            {10.91, 11975},
            {12.03, 11475},
            {16.33, 11000},
            {25.85, 11075},
    };

    public CompRapidReact() {
        configAll();
    }

    @Override
    public void configDrivetrain() {

    }

    @Override
    public void configIntake() {

    }

    @Override
    public void configConveyor() {

    }

    @Override
    public void configShooter() {

    }

    @Override
    public void configClimber() {

    }

    @Override
    public void defineLimits() {

    }

    @Override
    public void defineTargets() {

    }

    @Override
    public Target getPivotTarget(Climber.ClimberPivotPosition target) {
        return null;
    }

    @Override
    public Target getExtensionTarget(Climber.ClimberExtensionPosition target) {
        return null;
    }

    @Override
    public DrivetrainConfig getDrivetrainConfig() {
        return null;
    }

    @Override
    public IntakeConfig getIntakeConfig() {
        return null;
    }

    @Override
    public ConveyorConfig getConveyorConfig() {
        return null;
    }

    @Override
    public ShooterConfig getShooterConfig() {
        return null;
    }

    @Override
    public ClimberConfig getClimberConfig() {
        return null;
    }

}
