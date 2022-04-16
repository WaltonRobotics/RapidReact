package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.robotState.scoring.Shooting;
import frc.robot.util.EnhancedBoolean;

import static frc.robot.Constants.Shooter.kRecoveryToleranceRawUnits;
import static frc.robot.Constants.SmartDashboardKeys.kFlywheelOnTargetKey;
import static frc.robot.RobotContainer.godSubsystem;

public class WaitForCargoShot extends CommandBase {

    private final EnhancedBoolean flywheelOnTarget = new EnhancedBoolean();
    private double setpointVelocity;
    private double currentVelocity;

    @Override
    public void initialize() {
        flywheelOnTarget.set(false);
    }

    @Override
    public void execute() {
        setpointVelocity = godSubsystem.getCurrentTargetFlywheelVelocity();
        currentVelocity = godSubsystem.getShooter().getFlywheelVelocityNU();

        if (godSubsystem.getCurrentState() instanceof Shooting) {
            setpointVelocity = godSubsystem.getCurrentTargetFlywheelVelocity();
            currentVelocity = godSubsystem.getShooter().getFlywheelVelocityNU();

            flywheelOnTarget.set(Math.abs(setpointVelocity - currentVelocity) <= kRecoveryToleranceRawUnits);
        } else {
            flywheelOnTarget.set(false);
        }

        SmartDashboard.putBoolean(kFlywheelOnTargetKey, flywheelOnTarget.get());
    }

    @Override
    public boolean isFinished() {
        return flywheelOnTarget.isFallingEdge() && (currentVelocity < setpointVelocity);
    }

}
