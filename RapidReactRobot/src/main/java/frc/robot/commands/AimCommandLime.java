package frc.robot.commands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Limelight.kAlignmentPipeline;
import static frc.robot.RobotContainer.aimButton;
import static frc.robot.RobotContainer.leftJoystick;

public class AimCommandLime extends CommandBase {
    Drivetrain sdrivetrain = new Drivetrain();

    @Override
    public void execute() {
        float kP = -0.1f;
        float min_command = 0.05f;

        double tx = LimelightHelper.getTX();

        if(aimButton.get()){
            double heading_error = -tx;
            double steering_adjust = 0.0f;

            if(tx > 1.0){
                steering_adjust = kP*heading_error - min_command;
            }
            else if (tx < 1.0){
                steering_adjust = kP*heading_error + min_command;
            }
            sdrivetrain.move(0,0,steering_adjust,true);
        }
    }




//    {
//        float heading_error = -tx;
//        float steering_adjust = 0.0f;
//        if (tx > 1.0)
//        {
//            steering_adjust = Kp*heading_error - min_command;
//        }
//        else if (tx < 1.0)
//        {
//            steering_adjust = Kp*heading_error + min_command;
//        }
//        left_command += steering_adjust;
//        right_command -= steering_adjust;
//    }

}
