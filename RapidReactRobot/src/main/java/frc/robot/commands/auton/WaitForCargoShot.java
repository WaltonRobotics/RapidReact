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
    public void initialize() {
        flywheelOnTarget.set(false);
    }

    @Override
    public void execute() {
        if (godSubsystem.getCurrentState() instanceof Shooting) {
            double setpointVelocity = godSubsystem.getCurrentTargetFlywheelVelocity();
            double currentVelocity = godSubsystem.getShooter().getFlywheelVelocityNU();

            flywheelOnTarget.set(Math.abs(setpointVelocity - currentVelocity) <= kShootingToleranceRawUnits);
        } else {
            flywheelOnTarget.set(false);
        }

        SmartDashboard.putBoolean("Flywheel on target", flywheelOnTarget.get());
    }

    @Override
    public boolean isFinished() {
        return flywheelOnTarget.isFallingEdge();
    }

}
