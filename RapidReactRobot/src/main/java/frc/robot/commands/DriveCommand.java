package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

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
        double vx = forward * drivetrain.getConfig().getMaxSpeedMetersPerSecond();
        double vy = strafe * drivetrain.getConfig().getMaxSpeedMetersPerSecond();
        double omega = yaw * drivetrain.getConfig().getMaxOmega();

        drivetrain.move(vx, vy, omega, true);
    }

}
