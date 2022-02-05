package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.RobotContainer.controllerConfig;
import static frc.robot.RobotContainer.godSubsystem;

public class DriveCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    public DriveCommand() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        double forward = controllerConfig.getForwardScale().apply(controllerConfig.getForward());
        double strafe = controllerConfig.getForwardScale().apply(controllerConfig.getStrafe());
        double yaw = controllerConfig.getForwardScale().apply(controllerConfig.getYaw());
        double vx = forward * drivetrain.getConfig().kMaxSpeedMetersPerSecond;
        double vy = strafe * drivetrain.getConfig().kMaxSpeedMetersPerSecond;
        double omega = yaw * drivetrain.getConfig().kMaxOmega;

        drivetrain.move(vx, vy, omega, true);
    }

}
