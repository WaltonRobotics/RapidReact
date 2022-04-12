package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

import static frc.robot.RobotContainer.godSubsystem;

public class ShootCargo extends SequentialCommandGroup {

    public ShootCargo(int numberOfBalls, double timeout) {
        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToShoot(true)));

        for (int i = 0; i < numberOfBalls; i++) {
            addCommands(new WaitForCargoShot().withTimeout(timeout));
        }

        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToShoot(false)));
    }

    public ShootCargo(int numberOfBalls) {
        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToShoot(true)));

        for (int i = 0; i < numberOfBalls; i++) {
            addCommands(new WaitForCargoShot());
        }

        addCommands(new InstantCommand(() -> godSubsystem.setDoesAutonNeedToShoot(false)));
    }

}
