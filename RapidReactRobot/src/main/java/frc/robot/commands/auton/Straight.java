package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.godSubsystem;

public class Straight extends CommandBase {

    public Straight() {
        addRequirements(godSubsystem.getDrivetrain());
    }

    @Override
    public void execute() {
        godSubsystem.getDrivetrain().setModuleStates(new SwerveModuleState(Units.feetToMeters(6.0),
                Rotation2d.fromDegrees(0)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }

}
