package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Paths.RoutineOne.gammaMoveOffTarmac;
import static frc.robot.Paths.RoutineThree.alphaPickUpA;
import static frc.robot.RobotContainer.godSubsystem;

public enum AutonRoutine {

    DO_NOTHING("Doing Nothing", new SequentialCommandGroup(
    )),

    ROUTINE_ONE("Taxi from tarmac from gamma", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(gammaMoveOffTarmac.getInitialPose())),
            new SwerveTrajectoryCommand(Paths.RoutineOne.gammaMoveOffTarmac)
    )),

    ROUTINE_TWO("", new SequentialCommandGroup(
    )),

    ROUTINE_THREE("Start from alpha, shoot, pick up ball A, pick shoot 2 balls", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose())),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //SHOOT 2 Balls
    )),

    ROUTINE_FIVE_B("Start from alpha, shoot 1, pick up ball A, pick up ball B, shoot", new SequentialCommandGroup(

    ))

    ;

    private final String description;
    private final CommandBase commandGroup;

    AutonRoutine(String description, CommandBase commandGroup) {
        this.description = description;
        this.commandGroup = commandGroup;
    }

    public String getDescription() {
        return description;
    }

    public CommandBase getCommandGroup() {
        return commandGroup;
    }

    @Override
    public String toString() {
        return name() + ": " + description;
    }
}
