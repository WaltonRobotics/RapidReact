package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Limelight.kAlignmentPipeline;
import static frc.robot.Constants.Limelight.kMaximumLEDWaitTimeSeconds;
import edu.wpi.first.math.controller.ProfiledPIDController;

public class AimCommand extends SequentialCommandGroup {
    private Drivetrain sDrivetrain = new Drivetrain();
    public AimCommand(){
        addCommands(
                new InstantCommand(() -> LimelightHelper.setLEDMode(true)),
                new InstantCommand(() -> LimelightHelper.setPipeline(kAlignmentPipeline)),
                new TurnToAngle(() -> sDrivetrain.getHeading().getDegrees() - LimelightHelper.getTX())
        );
    }

}
