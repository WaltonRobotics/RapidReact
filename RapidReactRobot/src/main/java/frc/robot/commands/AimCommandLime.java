package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.LimelightHelper;

import static frc.robot.RobotContainer.*;

public class AimCommandLime extends CommandBase {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    public AimCommandLime() {
        addRequirements(drivetrain);
    }

    @Override
    public void execute() {
        float kP = 0.1f;
        float min_command = 0.05f;

        double tx = -LimelightHelper.getTX();

        double heading_error = -tx;
        double steering_adjust = 0.0f;

        if(tx > 1.0){
            steering_adjust = kP*heading_error - min_command;
        }
        else if (tx < 1.0){
            steering_adjust = kP*heading_error + min_command;
        }

        drivetrain.move(0,0,steering_adjust,false);
    }

}
