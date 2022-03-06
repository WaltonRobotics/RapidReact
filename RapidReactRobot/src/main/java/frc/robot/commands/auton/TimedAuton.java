package frc.robot.commands.auton;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Robot;

import static edu.wpi.first.wpilibj.Timer.getFPGATimestamp;

public class TimedAuton extends SequentialCommandGroup {

    private double startTime;

    public TimedAuton(Command... commands) {
        addCommands(commands);
    }

    @Override
    public void initialize() {
        startTime = getFPGATimestamp();
        super.initialize();
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("Time to complete auton " + (getFPGATimestamp() - startTime));
        SmartDashboard.putNumber("Auton time", (getFPGATimestamp() - startTime));
        super.end(interrupted);
    }

    @Override
    public boolean isFinished() {
        return super.isFinished() || getFPGATimestamp() - startTime >= 15;
    }
}














