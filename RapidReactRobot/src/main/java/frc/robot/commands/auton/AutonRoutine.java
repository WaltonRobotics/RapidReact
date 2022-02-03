package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;

import static frc.robot.Paths.RoutineOne.gammaMoveOffTarmac;
import static frc.robot.Paths.RoutineThree.alphaPickUpA;
import static frc.robot.RobotContainer.godSubsystem;

public enum AutonRoutine {

    ROUTINE_ONE("Taxi from tarmac from gamma", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new SwerveTrajectoryCommand(Paths.RoutineOne.gammaMoveOffTarmac)
    )),

    ROUTINE_TWO("", new SequentialCommandGroup(
    )),

    ROUTINE_THREE("Start from alpha, pick up ball A, shoot 2 balls", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),   //Do I need to zero intake sensors?
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose())),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //SHOOT 2 Balls
    ))

    ;

    private final String mDescription;
    private final CommandBase mCommandGroup;

    AutonRoutine(String description, CommandBase commandGroup) {
        this.mDescription = description;
        this.mCommandGroup = commandGroup;
    }

    public String getDescription() {
        return mDescription;
    }

    public CommandBase getCommandGroup() {
        return mCommandGroup;
    }

    @Override
    public String toString() {
        return name() + ": " + mDescription;
    }
}
