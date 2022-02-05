package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;
import frc.robot.subsystems.Drivetrain;

import static frc.robot.Paths.RoutineFiveB.ballAtoBallB;
import static frc.robot.Paths.RoutineFiveB.ballBtoBallG;
import static frc.robot.Paths.RoutineOne.gammaMoveOffTarmac;
import static frc.robot.Paths.RoutineSix.ballCtoBallG;
import static frc.robot.Paths.RoutineSix.gammaPickUpC;
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

    ROUTINE_FOUR_A("Start from beta, pick up ball B, shoot 2 balls, pick up ball(s) G, move in and shoot",
            new SequentialCommandGroup(

            )),

    ROUTINE_FIVE_B("Start from alpha, shoot 1, pick up ball A, pick up ball B, shoot", new SequentialCommandGroup(
            //SHOOT
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            //SHOOT
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose())),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),

            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(ballAtoBallB.getInitialPose())),
            new SwerveTrajectoryCommand(ballAtoBallB),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //SHOOT (may need to move closer)
    )),

    ROUTINE_FIVE_C("Routine_FIVE_B, move to pick up ball G, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            //SHOOT ONE
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose())),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(ballAtoBallB.getInitialPose())),
            new SwerveTrajectoryCommand(ballAtoBallB),
            //SHOOT TWO
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(ballBtoBallG.getInitialPose())),
            new SwerveTrajectoryCommand(ballBtoBallG),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //MOVE IN & SHOOT 1 or 2(moving in distance not determined, make path for that)
    )),

    ROUTINE_SIX("Start from gamma, pick up ball C, shoot 2, pick up ball G, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(gammaPickUpC.getInitialPose())),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(gammaPickUpC),
            //SHOOT 2
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(ballCtoBallG.getInitialPose())),
            new SwerveTrajectoryCommand(ballCtoBallG),
            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //MOVE IN & Shoot 1 or 2
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
