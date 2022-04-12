package frc.robot.commands.auton;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.RobotContainer.godSubsystem;

public class BarfBall extends SequentialCommandGroup {

    public BarfBall(int numberOfBalls, double timeout) {
        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToBarf(true)));

        for (int i = 0; i < numberOfBalls; i++) {
            addCommands(new WaitForCargoShot().withTimeout(timeout));
        }

        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToBarf(false)));
    }

}
