package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.robotState.scoring.Shooting;
import frc.robot.util.EnhancedBoolean;

import static frc.robot.Constants.Shooter.kShootingToleranceRawUnits;
import static frc.robot.RobotContainer.godSubsystem;

public class WaitForCargoShot extends CommandBase {

    private final EnhancedBoolean flywheelOnTarget = new EnhancedBoolean();

    @Override
    public boolean isFinished() {
        if (godSubsystem.getCurrentState() instanceof Shooting) {
            double setpointVelocity = godSubsystem.getCurrentTargetFlywheelVelocity();
            double currentVelocity = godSubsystem.getShooter().getFlywheelVelocityNU();

            flywheelOnTarget.set(Math.abs(setpointVelocity - currentVelocity) < kShootingToleranceRawUnits);

            SmartDashboard.putBoolean("Flywheel on target", flywheelOnTarget.get());

            return flywheelOnTarget.isFallingEdge();
        } else {
            return false;
        }
    }

}
