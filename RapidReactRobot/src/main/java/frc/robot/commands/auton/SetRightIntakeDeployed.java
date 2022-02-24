package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.Intake;

import static frc.robot.RobotContainer.godSubsystem;

public class SetRightIntakeDeployed extends SequentialCommandGroup {

    private final Intake intake = godSubsystem.getIntake();

    public SetRightIntakeDeployed(boolean deployed, double wait) {
        addCommands(
                new InstantCommand(() -> intake.setRightIntakeDeployStateDemand(deployed)),
                new WaitCommand(wait)
        );
    }

    public SetRightIntakeDeployed(boolean deployed) {
        this(deployed, 0);
    }

}
