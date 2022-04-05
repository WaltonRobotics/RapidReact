package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.RobotContainer.godSubsystem;
public class BarfBall extends CommandBase {
    private final Timer timer = new Timer();
    private final double totalTimeSeconds;

    public BarfBall(double timeSeconds){
        totalTimeSeconds = timeSeconds;
    }

    @Override
    public void initialize() {
        timer.reset();
        timer.start();
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
