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

    FIVE_FEET_FORWARD("moves forward five feet starting 1 meter (3.28084 ft) from wall", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(fiveFeetForward.getInitialPose(), fiveFeetForward.getInitialState())),
            new SwerveTrajectoryCommand(fiveFeetForward)
    )),

    S_CURVE_FORWARD("forward curve to test PID controllers", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(sCurveForward.getInitialPose(), sCurveForward.getInitialState())),
            new SwerveTrajectoryCommand(sCurveForward)
    )),

    //FOR REVERSAL TRAJECTORIES: add 180 to negative angles, subtract 180 from positive angles, size of heading doesn't matter
    S_CURVE_BACKWARD("backward curve to test PID controllers", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(sCurveBackward.getInitialPose(), sCurveBackward.getInitialState())),
            new SwerveTrajectoryCommand(sCurveBackward)
    )),

    ROUTINE_ONE("Taxi from tarmac from gamma", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(gammaMoveOffTarmac.getInitialPose(), gammaMoveOffTarmac.getInitialState())),
            new SwerveTrajectoryCommand(Paths.RoutineOne.gammaMoveOffTarmac)
    )),

    ROUTINE_TWO("start from beta, shoot 1 ball, move backward off tarmac", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            //SHOOT 1 BALL
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(betaBackward.getInitialPose(), betaBackward.getInitialState())),
            new SwerveTrajectoryCommand(betaBackward)
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //distance off is not precise

    )),

    ROUTINE_THREE("Start from alpha, shoot, pick up ball A, pick shoot 2 balls", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose(), alphaPickUpA.getInitialState())),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA)
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //SHOOT 2 Balls
    )),

    //FOUR_B is the same except picks up 2 balls from G
    ROUTINE_FOUR_A("Start from beta, pick up ball B, shoot 2 balls, pick up ball(s) G, move in and shoot",
            new SequentialCommandGroup(
                    new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//                    new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
                    new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(betaPickUpB.getInitialPose(), betaPickUpB.getInitialState())),
                    new SwerveTrajectoryCommand(betaPickUpB),
//                    new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0)),
                    //SHOOT 2 Balls (may have to rotate)
//                    new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
                    new SwerveTrajectoryCommand(ballBtoBallG)
//                    new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
                    //MOVE IN AND SHOOT

            )),
    ROUTINE_FIVE_A("Start from alpha, Shoot 1, pick up ball A, shoot 1", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            //SHOOT 1 Ball
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose(), alphaPickUpA.getInitialState())),
            new SwerveTrajectoryCommand(alphaPickUpA)
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //SHOOT 1 Ball (may have to rotate)
    )),

    ROUTINE_FIVE_B("Start from alpha, shoot 1, pick up ball A, pick up ball B, shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            //SHOOT 1 Ball
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose(), alphaPickUpA.getInitialState())),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),

            new SwerveTrajectoryCommand(ballAtoBallB)
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //SHOOT (may need to move closer)
    )),

    ROUTINE_FIVE_C("Routine_FIVE_B, move to pick up ball G, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            //SHOOT ONE
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose(), alphaPickUpA.getInitialState())),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new SwerveTrajectoryCommand(ballAtoBallB),
            //SHOOT TWO
            new SwerveTrajectoryCommand(ballBtoBallG)
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //MOVE IN & SHOOT 1 or 2(moving in distance not determined, make path for that)
    )),

    ROUTINE_FIVE_D("ROUTINE_FIVE_B, move to pick up ball C, shoot 1", new SequentialCommandGroup(
            //SHOOT
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            //SHOOT
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose(), alphaPickUpA.getInitialState())),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(alphaPickUpA),

            new SwerveTrajectoryCommand(ballAtoBallB)
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //TODO: Pick up Ball C
            //SHOOT (may need to move closer)
    )),

    ROUTINE_SIX_A("Start from alpha, pick up ball a, shoot 2, move to ballG, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(alphaPickUpA.getInitialPose(), alphaPickUpA.getInitialState())),
            //intake
            new SwerveTrajectoryCommand(alphaPickUpA),
            //shoot 2
            new SwerveTrajectoryCommand(ballAtoballG)
            //move in & shoot
    )),

    ROUTINE_SIX_G("Start from gamma, pick up ball C, shoot 2, pick up ball G, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new InstantCommand(() -> godSubsystem.getDrivetrain().resetPose(gammaPickUpC.getInitialPose(), gammaPickUpC.getInitialState())),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(gammaPickUpC),
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0)),
            //SHOOT 2
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(8.0)),
            new SwerveTrajectoryCommand(ballCtoballG)
//            new InstantCommand(() -> godSubsystem.getIntake().setVoltage(0))
            //MOVE IN & Shoot 1 or 2
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
