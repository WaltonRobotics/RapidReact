package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.godSubsystem;

public class AlignAndShootCargo extends CommandBase {

    private final Timer timer = new Timer();
    private double totalTimeSeconds;

    public AlignAndShootCargo(double timeSeconds) {
        totalTimeSeconds = timeSeconds;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        godSubsystem.setDoesAutonNeedToAlignAndShoot(true);
    }

    @Override
    public void end(boolean interrupted) {
        godSubsystem.setDoesAutonNeedToAlignAndShoot(false);

        timer.stop();
    }

    @Override
    public boolean isFinished() {
        return timer.get() > totalTimeSeconds;
    }

}
