package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;

import static frc.robot.Paths.RoutineFiveB.ballAtoBallB;
import static frc.robot.Paths.RoutineFiveC.ballBtoBallG;
import static frc.robot.Paths.RoutineFourA.betaPickUpB;
import static frc.robot.Paths.RoutineOne.gammaMoveOffTarmac;
import static frc.robot.Paths.RoutineSixA.ballAtoballG;
import static frc.robot.Paths.RoutineSixG.ballCtoballG;
import static frc.robot.Paths.RoutineSixG.gammaPickUpC;
import static frc.robot.Paths.RoutineThree.alphaPickUpA;
import static frc.robot.Paths.RoutineTwo.betaBackward;
import static frc.robot.Paths.*;
import static frc.robot.RobotContainer.godSubsystem;

public enum AutonRoutine {

    DO_NOTHING("Doing Nothing", new SequentialCommandGroup(
    )),

    FIVE_FEET_FORWARD("Moves forward five feet", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(fiveFeetForward),
            new SwerveTrajectoryCommand(fiveFeetForward)
    )),

    S_CURVE_FORWARD("Forward curve to test PID controllers", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(sCurveForward),
            new SwerveTrajectoryCommand(sCurveForward)
    )),

    // For reversal trajectories: add 180 to negative angles, subtract 180 from positive angles, size of heading doesn't matter
    S_CURVE_BACKWARD("Backward curve to test PID controllers", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(sCurveBackward),
            new SwerveTrajectoryCommand(sCurveBackward)
    )),

    ROUTINE_ONE("Taxi from tarmac from gamma", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaMoveOffTarmac),
            new SwerveTrajectoryCommand(Paths.RoutineOne.gammaMoveOffTarmac)
    )),

    ROUTINE_TWO("Start from beta, shoot 1 ball, move backward off tarmac", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(betaBackward),
            new ShootCargo(3.0),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(betaBackward)
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
    )),

    ROUTINE_THREE("Start from alpha, shoot, pick up ball A, pick shoot 2 balls", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            new ShootCargo(3.0)
    )),

    //FOUR_B is the same except picks up 2 balls from G
    ROUTINE_FOUR_A("Start from beta, pick up ball B, shoot 2 balls, pick up ball(s) G, move in and shoot",
            new SequentialCommandGroup(
                    new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
                    new ResetPose(betaPickUpB),
//                    new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
                    new SwerveTrajectoryCommand(betaPickUpB),
//                    new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0)),
                    new ShootCargo(3.0),
//                    new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
                    new SwerveTrajectoryCommand(ballBtoBallG),
//                    new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
                    // Move in and shoot
                    new ShootCargo(3.0)
            )
    ),

    ROUTINE_FIVE_A("Start from alpha, Shoot 1, pick up ball A, shoot 1", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(3.0),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            new ShootCargo(3.0)
    )),

    ROUTINE_FIVE_B("Start from alpha, shoot 1, pick up ball A, pick up ball B, shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(3.0),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new SwerveTrajectoryCommand(ballAtoBallB),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            new ShootCargo(3.0)
            //SHOOT (may need to move closer)
    )),

    ROUTINE_FIVE_C("Routine_FIVE_B, move to pick up ball G, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(3.0),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new SwerveTrajectoryCommand(ballAtoBallB),
            new ShootCargo(3.0),
            new SwerveTrajectoryCommand(ballBtoBallG),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            new ShootCargo(3.0)
    )),

    ROUTINE_FIVE_D("ROUTINE_FIVE_B, move to pick up ball C, shoot 1", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(3.0),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new SwerveTrajectoryCommand(ballAtoBallB),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            // TODO: Pick up Ball C
            new ShootCargo(3.0)
    )),

    ROUTINE_SIX_A("Start from alpha, pick up ball a, shoot 2, move to ballG, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new ShootCargo(3.0),
            new SwerveTrajectoryCommand(ballAtoballG),
            new ShootCargo(3.0)
    )),

    ROUTINE_SIX_G("Start from gamma, pick up ball C, shoot 2, pick up ball G, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaPickUpC),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(gammaPickUpC),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0)),
            new ShootCargo(3.0),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(ballCtoballG),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            new ShootCargo(3.0)
    ));

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
