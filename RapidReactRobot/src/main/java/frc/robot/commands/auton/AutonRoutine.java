package frc.robot.commands.auton;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Paths;
import frc.robot.subsystems.Superstructure;

import static frc.robot.Paths.NewFiveBallRoutine.*;
import static frc.robot.Paths.RoutineEight.pickupG;
import static frc.robot.Paths.RoutineEight.pickupGFast;
import static frc.robot.Paths.RoutineFiveFull.*;
import static frc.robot.Paths.RoutineOne.gammaBackwards;
import static frc.robot.Paths.RoutineSixG.gammaPickUpC;
import static frc.robot.Paths.RoutineTwo.betaBackwards;
import static frc.robot.Paths.TestTrajectories.*;
import static frc.robot.Paths.TwoBallThrowRoutine.twoBall;
import static frc.robot.Paths.TwoBallThrowRoutine.twoBallThrow;
import static frc.robot.RobotContainer.godSubsystem;

public enum AutonRoutine {

    HALF_FOOT_BACKWARDS("Moves backwards 6 inches (it'll be off by .12 inches)", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(halfFootBackwards),
            new SwerveTrajectoryCommand(halfFootBackwards)
    )),

    DO_NOTHING("Do Nothing", new SequentialCommandGroup(
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
    S_CURVE_BACKWARD("Backward curve to test PID controllers", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(sCurveBackward),
            new SwerveTrajectoryCommand(sCurveBackward)
    )),

    SHOOT_ONE("Shoot one cargo", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaBackwards),
            new ShootCargoTimed(10.0)
    )),

    TAXI_FROM_TARMAC("Taxi from tarmac from gamma", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaBackwards),
            new SwerveTrajectoryCommand(Paths.RoutineOne.gammaBackwards)
    )),

    ONE_BALL("Start from beta, shoot 1 ball, move backward off tarmac", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(betaBackwards),
            new ShootCargoTimed(3.0),
            new SwerveTrajectoryCommand(betaBackwards)
    )),

//    TWO_BALL_B("Start from alpha, pick up ball A, turn to face target, shoot 2 balls", new TimedAuton(
//            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//            new ResetPose(alphaPickUpA),
//            new ShootCargo(3.0),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetRightIntakeDeployed(true),
//            new SwerveTrajectoryCommand(alphaPickUpA),
//            new SetRightIntakeDeployed(false),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new TurnToAngle(90),    //this angle may not be correct
//            new ShootCargo(3.0)
//    )),

    TWO_BALL("Start from gamma, pick up ball C, shoot 2", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaPickUpC),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true)),
            new SetLeftIntakeDeployed(true),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SwerveTrajectoryCommand(gammaPickUpC, Rotation2d.fromDegrees(90), true),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetLeftIntakeDeployed(false),
//            new TurnToAngle(90.0).withTimeout(2.0),
            new WaitCommand(2.0),
            new AlignAndShootCargoTimed(10.0)
    )),

    TWO_BALL_THROW("Start from gamma, pick up ballC, shoot 2, throw enemy ball into hanger", new TimedAuton(
            //normal two ball
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(twoBall),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true)),
            new SetLeftIntakeDeployed(true),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SwerveTrajectoryCommand(twoBall),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetLeftIntakeDeployed(false),
            new ShootCargo(2),
            //throw enemy ball
            new ParallelDeadlineGroup(
                    new SwerveTrajectoryCommand(twoBallThrow),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true)),
                            new SetLeftIntakeDeployed(true),
                            new WaitCommand(twoBallThrow.getTotalTimeSeconds()*0.5),  //spin up 50% through path
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true))
                    )
            ),
            new ParallelCommandGroup(
                    new ShootCargo(1),
                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
                    new SetLeftIntakeDeployed(false)
            )
    )),

//    //FOUR_B is the same except picks up 2 balls from G
//    ROUTINE_FOUR_A("Start from beta, pick up ball B, shoot 2 balls, pick up ball(s) G, move in and shoot",
//            new SequentialCommandGroup(
//                    new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//                    new ResetPose(betaPickUpB),
//                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//                    new SetLeftIntakeDeployed(true),
//                    new SwerveTrajectoryCommand(betaPickUpB),
//                    new SetLeftIntakeDeployed(false),
//                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//                    new ShootCargo(3.0),
//
//                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//                    new SetLeftIntakeDeployed(true),
//                    new SwerveTrajectoryCommand(ballBToBallG),
//
//                    new SetLeftIntakeDeployed(false),
//                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//                    new SwerveTrajectoryCommand(ballGToBallB),
//                    new ShootCargo(3.0)
//            )
//    ),

//    ROUTINE_FIVE_A("Start from alpha, Shoot 1, pick up ball A, shoot 1", new SequentialCommandGroup(
//            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//            new ResetPose(alphaPickUpA),
//            new ShootCargo(3.0),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetRightIntakeDeployed(true),
//            new SwerveTrajectoryCommand(alphaPickUpA),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new SetRightIntakeDeployed(false),
//            new ShootCargo(3.0)
//    )),
//
//    ROUTINE_FIVE_B("Start from alpha, shoot 1, pick up ball A, pick up ball B, move in to shoot", new TimedAuton(
//            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//            new ResetPose(alphaPickUpA),
//            new ShootCargo(2),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetRightIntakeDeployed(true),
//            new SwerveTrajectoryCommand(alphaPickUpA),
//            new SwerveTrajectoryCommand(alphaPickUpA2),
//            new SetRightIntakeDeployed(false),
//            new SetLeftIntakeDeployed(true),
//            new SwerveTrajectoryCommand(ballAToBallB),
//            new SwerveTrajectoryCommand(ballAToBallB2),
//            new SetRightIntakeDeployed(false),
//            new AlignAndShootCargo(3.0)
//    )),
//
//    ROUTINE_FIVE_C("Routine_FIVE_B, move to pick up ball G, move in to shoot", new TimedAuton(
//            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//            new ResetPose(alphaPickUpA),
//            new ShootCargo(1.5),
////            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
////            new SetRightIntakeDeployed(true),
//            new SwerveTrajectoryCommand(alphaPickUpA),
////            new SetRightIntakeDeployed(false),
////            new SetLeftIntakeDeployed(true),
//            new SwerveTrajectoryCommand(ballAToBallB),
////            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
////            new SetRightIntakeDeployed(false),
//            new ShootCargo(3.0),
////            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
////            new SetLeftIntakeDeployed(true),
//            new SwerveTrajectoryCommand(ballBToBallG),
////            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new SetLeftIntakeDeployed(false),
//            new SwerveTrajectoryCommand(ballGToBallB),
//            new ShootCargo(3.0)
//    )),
//
//    ROUTINE_FIVE_D("ROUTINE_FIVE_B, move to pick up ball C, shoot 1", new SequentialCommandGroup(
//            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//            new ResetPose(alphaPickUpA),
//            new ShootCargo(1.5),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetRightIntakeDeployed(true),
//            new SwerveTrajectoryCommand(alphaPickUpA),
//            new SetRightIntakeDeployed(false),
//
//            new SetLeftIntakeDeployed(true),
//            new SwerveTrajectoryCommand(ballAToBallB),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new SetRightIntakeDeployed(false),
//            new ShootCargo(3.0),
//
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetLeftIntakeDeployed(true),
//            new SwerveTrajectoryCommand(ballBToBallG),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new SetLeftIntakeDeployed(false),
//
//            new SwerveTrajectoryCommand(ballGToBallB),
//            new ShootCargo(3.0),
//
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetLeftIntakeDeployed(true),
//            new SwerveTrajectoryCommand(ballBToBallC),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new SetLeftIntakeDeployed(false),
//            new ShootCargo(3.0)
//
//    )),
//
//    ROUTINE_FIVE_E("shoot from alpha, pick up ball A & ball B, come in to make 'money shot'", new SequentialCommandGroup(
//
//            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//            new ResetPose(alphaPickUpA),
//            new ShootCargo(1.5),
//
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetRightIntakeDeployed(true),
//            new SwerveTrajectoryCommand(alphaPickUpA),
//            new SetRightIntakeDeployed(false),
//
//            new SetLeftIntakeDeployed(true),
//            new SwerveTrajectoryCommand(ballAToBallB),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new SetRightIntakeDeployed(false),
//
//            new SwerveTrajectoryCommand(ballBToMoneyShot),
//            new ShootCargo(3.0)
//    )),

    THREE_BALL("ROUTINE_FIVE_B in one complete path", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(routineFiveBFullFast),
            new ParallelDeadlineGroup(
                    new ShootCargoTimed(2.0),
                    new SequentialCommandGroup(
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
                            new SetLeftIntakeDeployed(true),
                            new SetRightIntakeDeployed(true)
                    )
            ),
            new SwerveTrajectoryCommand(routineFiveBFull),
            new SetLeftIntakeDeployed(false),
            new SetRightIntakeDeployed(false),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new AlignAndShootCargo(3)
    )),

    TEST_ROUTINE("test", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(routineFiveBFull),
            new SwerveTrajectoryCommand(routineFiveBFull)
    )),

//    ROUTINE_SIX_A("Start from alpha, pick up ball a, shoot 2, move to ballG, move in to shoot", new SequentialCommandGroup(
//            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//            new ResetPose(alphaPickUpA),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetRightIntakeDeployed(true),
//            new SwerveTrajectoryCommand(alphaPickUpA),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new SetRightIntakeDeployed(false),
//            new TurnToAngle(75),    //angle may not be correct
//            new ShootCargo(3.0),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetRightIntakeDeployed(true),
//            new SwerveTrajectoryCommand(ballAToBallG),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new SetRightIntakeDeployed(false),
//            new SwerveTrajectoryCommand(ballGToShoot),
//            new ShootCargo(3.0)
//    )),

    THREE_BALL_PICKUP_TWO("3 ball auton, pick up 2", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(routineFiveBFull),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true)),
            new ParallelDeadlineGroup(
                    new ShootCargo(1, 1.25),
                    new SequentialCommandGroup(
                            new SetLeftIntakeDeployed(true),
                            new SetRightIntakeDeployed(true),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true))
                    )
            ),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(false)),
            new ParallelDeadlineGroup(
                    new SwerveTrajectoryCommand(routineFiveBFullFast),
                    new ParallelCommandGroup(
                            new WaitCommand(routineFiveBFullFast.getTotalTimeSeconds() * 0.75),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true))
                    )
            ),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new AlignAndShootCargo(2, 3.5),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(false)),
            new SetRightIntakeDeployed(false),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SwerveTrajectoryCommand(pickupG),
            new WaitCommand(5.0),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false))
    )),

    FIVE_BALL("Five ball auton", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(routineFiveBFullFast),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true)),
            new ParallelDeadlineGroup(
                    new ShootCargoTimed(1.25),
                    new SequentialCommandGroup(
                            new SetLeftIntakeDeployed(true),
                            new SetRightIntakeDeployed(true),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true))
                    )
            ),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(false)),
            new ParallelDeadlineGroup(
                    new SwerveTrajectoryCommand(routineFiveBFullFast),
                    new ParallelCommandGroup(
                            new WaitCommand(routineFiveBFullFast.getTotalTimeSeconds() * 0.75),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true))
                    )
            ),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new AlignAndShootCargo(2, 5),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(false)),
            new SetRightIntakeDeployed(false),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SwerveTrajectoryCommand(pickupGFast),
            new WaitCommand(0.5),
            new SwerveTrajectoryCommand(pickupGShoot)
    )),

    NEW_FIVE_BALL("Pick up & shoot 2, shoot 1, pick up & shoot 2", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(fiveBall1),
            new SetRightIntakeDeployed(true),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new ParallelDeadlineGroup(
                    new SwerveTrajectoryCommand(fiveBall1, Rotation2d.fromDegrees(90), true),
                    new SequentialCommandGroup(
                            new WaitCommand(fiveBall1.getTotalTimeSeconds() * 0.75),  // spin up 75% through path
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true)),
                            new SetRightIntakeDeployed(false),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false))
                    )),
            new ParallelCommandGroup(
                    new ShootCargo(3, 1.0), //intake & shoot ballB while shooting
                    new SequentialCommandGroup(
//                            new WaitCommand(0.5),
                            new SetLeftIntakeDeployed(true),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
                            new WaitCommand(1.5) // Wait for intaking ball
                    )
            ),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(false)),
            new SwerveTrajectoryCommand(fiveBall2Full),
            //new SwerveTrajectoryCommand(fiveBall2),
            //new SwerveTrajectoryCommand(fiveBall2Half),
            new WaitCommand(1.0),   // wait for human player
            new ParallelDeadlineGroup(
                    new SwerveTrajectoryCommand(fiveBall3),
                    new SequentialCommandGroup(
                            new WaitCommand(fiveBall3.getTotalTimeSeconds() * 0.5),  // spin up 50% through path
                            new SetLeftIntakeDeployed(false),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true))
                    )
            ),
            new AlignAndShootCargo(2, 5),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(false))
    ));
    
//
//    ROUTINE_SEVEN("Start from gamma, pick up ball C, shoot 2, pick up ball B, pick up ball A, shoot 2", new SequentialCommandGroup(
//            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
//            new ResetPose(gammaPickUpC),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
//            new SetLeftIntakeDeployed(true),
//            new SwerveTrajectoryCommand(gammaPickUpC),
//            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
//            new SetLeftIntakeDeployed(false),
//            new SwerveTrajectoryCommand(ballCToShoot),
//            new ShootCargo(3.0),
//            new SwerveTrajectoryCommand(ballCToBallB),
//            new SwerveTrajectoryCommand(ballBToBallA),
//            new SwerveTrajectoryCommand(ballAToShoot),
//            new ShootCargo(3.0)
//    ));

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
