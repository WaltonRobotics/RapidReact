package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.util.EnhancedBoolean;

import static frc.robot.Constants.Shooter.kShootingToleranceRawUnits;
import static frc.robot.RobotContainer.godSubsystem;

public class WaitForCargoShot extends CommandBase {

    private EnhancedBoolean flywheelOnTarget = new EnhancedBoolean();

    @Override
    public boolean isFinished() {
        double setpointVelocity = godSubsystem.getCurrentTargetFlywheelVelocity();
        double currentVelocity = godSubsystem.getShooter().getFlywheelVelocityNU();

        flywheelOnTarget.set(Math.abs(setpointVelocity - currentVelocity) < kShootingToleranceRawUnits);

        return flywheelOnTarget.isFallingEdge();
    }

}
