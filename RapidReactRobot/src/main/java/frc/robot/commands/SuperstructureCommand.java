package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

import static frc.robot.RobotContainer.godSubsystem;

public class SuperstructureCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Climber climber;

    // State machine states:
    // Disabled
    // Initializing
    // Idle
    // Intaking
    // Outtaking
    // AdjustingHood
    // AligningAndSpinningUp
    // Shooting
    // SpinningUp

    public SuperstructureCommand() {
        addRequirements(godSubsystem);

        drivetrain = godSubsystem.getDrivetrain();
        intake = godSubsystem.getIntake();
        conveyor = godSubsystem.getConveyor();
        shooter = godSubsystem.getShooter();
        climber = godSubsystem.getClimber();
    }

}
