package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;

import static frc.robot.RobotContainer.godSubsystem;

public class AlignAndShootCargoTimed extends CommandBase {

    private final Timer timer = new Timer();
    private final double totalTimeSeconds;

    public AlignAndShootCargoTimed(double timeSeconds) {
        totalTimeSeconds = timeSeconds;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();

        godSubsystem.setDoesAutonNeedToAlignAndShoot(true);
    }

    @Override
    public void execute() {
        godSubsystem.setDoesAutonNeedToAlignAndShoot(true);
    }

    @Override
    public void end(boolean interrupted) {
        godSubsystem.setDoesAutonNeedToAlignAndShoot(false);
    }

    @Override
    public boolean isFinished() {
        return timer.hasElapsed(totalTimeSeconds);
    }

}
