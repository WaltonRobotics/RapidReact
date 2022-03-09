package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.godSubsystem;

public class ShootCargo extends CommandBase {

    private final Timer timer = new Timer();
    private double totalTimeSeconds;

    public ShootCargo(double timeSeconds) {
        totalTimeSeconds = timeSeconds;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        godSubsystem.setDoesAutonNeedToShoot(true);
    }

    @Override
    public void execute() {
        godSubsystem.setDoesAutonNeedToShoot(true);
    }

    @Override
    public void end(boolean interrupted) {
        godSubsystem.setDoesAutonNeedToShoot(false);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(totalTimeSeconds);
    }

}
