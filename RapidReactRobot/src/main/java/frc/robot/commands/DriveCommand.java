package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.RobotContainer.godSubsystem;

public class DriveCommand extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    public DriveCommand() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {

    }

}
