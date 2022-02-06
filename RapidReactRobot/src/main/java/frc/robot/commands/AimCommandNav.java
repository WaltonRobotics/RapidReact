package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Limelight.kAlignmentPipeline;
import static frc.robot.RobotContainer.godSubsystem;

public class AimCommandNav extends SequentialCommandGroup {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    public AimCommandNav() {
        addCommands(
                new InstantCommand(() -> LimelightHelper.setLEDMode(true)),
                new InstantCommand(() -> LimelightHelper.setPipeline(kAlignmentPipeline)),
                new TurnToAngle(() -> drivetrain.getHeading().getDegrees() - LimelightHelper.getTX())
        );
    }

}
