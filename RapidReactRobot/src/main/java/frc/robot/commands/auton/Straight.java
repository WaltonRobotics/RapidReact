package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.WaltSwerveModule;

import static frc.robot.RobotContainer.godSubsystem;

public class Straight extends CommandBase {

    private final double velocity;

    public Straight(double velocity) {
        this.velocity = velocity;
        addRequirements(godSubsystem.getDrivetrain());
    }

    @Override
    public void execute() {
//        godSubsystem.getDrivetrain().setModuleStates(new SwerveModuleState(velocity,
//                Rotation2d.fromDegrees(0)));

        for (WaltSwerveModule swerveModule : godSubsystem.getDrivetrain().getSwerveModules()) {
            swerveModule.setAbsoluteAzimuthRotation2d(Rotation2d.fromDegrees(0));
            swerveModule.setDriveClosedLoopVelocityNU(5000);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
