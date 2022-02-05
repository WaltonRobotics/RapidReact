package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Constants.SmartDashboardKeys.kDrivetrainSetpointAngleDegreesKey;
import static frc.robot.RobotContainer.godSubsystem;

public class RotateModulesToAngle extends SequentialCommandGroup {

    private final Drivetrain drivetrain = godSubsystem.getDrivetrain();

    public RotateModulesToAngle() {
        addRequirements(drivetrain);

        addCommands(
                new RunCommand(() -> {
                    drivetrain.setModuleStates(new SwerveModuleState(
                            0.0,
                            Rotation2d.fromDegrees(SmartDashboard.getNumber(kDrivetrainSetpointAngleDegreesKey, 0.0))
                    ));
                }).withTimeout(5.0)
        );
    }

}
