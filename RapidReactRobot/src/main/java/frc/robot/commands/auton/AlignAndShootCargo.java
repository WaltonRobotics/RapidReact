package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.RobotContainer.godSubsystem;

public class AlignAndShootCargo extends SequentialCommandGroup {

    public AlignAndShootCargo(int numberOfBalls, double timeoutPerBall) {
        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToAlignAndShoot(true)));

        for (int i = 0; i < numberOfBalls; i++) {
            addCommands(new WaitForCargoShot().withTimeout(timeoutPerBall));
        }

        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToAlignAndShoot(false)));
    }

    public AlignAndShootCargo(int numberOfBalls) {
        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToAlignAndShoot(true)));

        for (int i = 0; i < numberOfBalls; i++) {
            addCommands(new WaitForCargoShot());
        }

        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToAlignAndShoot(false)));
    }

}
