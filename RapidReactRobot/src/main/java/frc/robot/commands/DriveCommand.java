package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.SwerveDriveConfig.kMaxOmega;
import static frc.robot.Constants.SwerveDriveConfig.kMaxSpeedMetersPerSecond;
import static frc.robot.RobotContainer.controllerConfig;
import static frc.robot.RobotContainer.godSubsystem;

public class DriveCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    public DriveCommand() {
        addRequirements(drivetrain);

        SmartDashboard.putNumber("Minimum omega command", 0.1);
    }

    @Override
    public void execute() {
        double forward = controllerConfig.getForwardScale().apply(controllerConfig.getForward());
        double strafe = controllerConfig.getForwardScale().apply(controllerConfig.getStrafe());
        double yaw = controllerConfig.getForwardScale().apply(controllerConfig.getYaw());
        double vx = forward * kMaxSpeedMetersPerSecond;
        double vy = strafe * kMaxSpeedMetersPerSecond;
        double omega = yaw * kMaxOmega;

        drivetrain.move(vx, vy, omega, true);
    }

}
