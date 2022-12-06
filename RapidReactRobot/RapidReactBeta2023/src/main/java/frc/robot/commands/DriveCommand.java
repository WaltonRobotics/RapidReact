package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import static frc.robot.Constants.SmartDashboardKeys.kDrivetrainIsFieldRelativeKey;
import static frc.robot.Constants.SmartDashboardKeys.kDrivetrainIsPositionalRotationKey;
import static frc.robot.RobotContainer.godSubsystem;

public class DriveCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    private static boolean isFieldRelative = true;
    private boolean isPositionalRotation = false;


    public DriveCommand() {
        addRequirements(drivetrain);

        SmartDashboard.putNumber("Minimum omega command", 0.1);
    }

    @Override
    public void execute() {
        isFieldRelative = !isFieldRelative;


        SmartDashboard.putBoolean(kDrivetrainIsFieldRelativeKey, isFieldRelative);
        SmartDashboard.putBoolean(kDrivetrainIsPositionalRotationKey, isPositionalRotation);

        double vx = godSubsystem.getVx();
        double vy = godSubsystem.getVy();
        double omega = godSubsystem.getOmega();
        drivetrain.move(vx, vy, omega, isFieldRelative);
        }
    }