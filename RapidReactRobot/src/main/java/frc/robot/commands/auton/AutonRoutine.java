package frc.robot.commands.auton;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Paths;

import static frc.robot.Paths.RoutineFiveB.ballAToBallB;
import static frc.robot.Paths.RoutineFiveC.ballBToBallG;
import static frc.robot.Paths.RoutineFiveD.ballBToBallC;
import static frc.robot.Paths.RoutineFiveE.ballBToMoneyShot;
import static frc.robot.Paths.RoutineFiveFull.routineFiveBFull;
import static frc.robot.Paths.RoutineFourA.*;
import static frc.robot.Paths.RoutineOne.gammaBackwards;
import static frc.robot.Paths.RoutineSeven.*;
import static frc.robot.Paths.RoutineSixG.*;
import static frc.robot.Paths.RoutineThree.alphaPickUpA;
import static frc.robot.Paths.RoutineTwo.betaBackwards;
import static frc.robot.Paths.TestTrajectories.*;
import static frc.robot.RobotContainer.godSubsystem;

public enum AutonRoutine {

    // Completed: all tests paths, 1, 2, 3, 4, 5a, 5b,

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
    S_CURVE_BACKWARD("Backward curve to test PID controllers", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(sCurveBackward),
            new SwerveTrajectoryCommand(sCurveBackward)
    )),

    ROUTINE_ONE("Taxi from tarmac from gamma", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaBackwards),
            new SwerveTrajectoryCommand(Paths.RoutineOne.gammaBackwards)
    )),

    ROUTINE_TWO("Start from beta, shoot 1 ball, move backward off tarmac", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(betaBackwards),
            new ShootCargo(3.0),
            new SwerveTrajectoryCommand(betaBackwards)
    )),

    ROUTINE_THREE("Start from alpha, pick up ball A, turn to face target, shoot 2 balls", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(3.0),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetRightIntakeDeployed(true),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new SetRightIntakeDeployed(false),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new TurnToAngle(90),    //this angle may not be correct
            new ShootCargo(3.0)
    )),

    //FOUR_B is the same except picks up 2 balls from G
    ROUTINE_FOUR_A("Start from beta, pick up ball B, shoot 2 balls, pick up ball(s) G, move in and shoot",
            new SequentialCommandGroup(
                    new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
                    new ResetPose(betaPickUpB),
                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
                    new SetLeftIntakeDeployed(true),
                    new SwerveTrajectoryCommand(betaPickUpB),
                    new SetLeftIntakeDeployed(false),
                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
                    new ShootCargo(3.0),

                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
                    new SetLeftIntakeDeployed(true),
                    new SwerveTrajectoryCommand(ballBToBallG),

                    new SetLeftIntakeDeployed(false),
                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
                    new SwerveTrajectoryCommand(ballGToBallB),
                    new ShootCargo(3.0)
            )
    ),

    ROUTINE_FIVE_A("Start from alpha, Shoot 1, pick up ball A, shoot 1", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(3.0),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetRightIntakeDeployed(true),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetRightIntakeDeployed(false),
            new ShootCargo(3.0)
    )),

    ROUTINE_FIVE_B("Start from alpha, shoot 1, pick up ball A, pick up ball B, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(1.5),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetRightIntakeDeployed(true),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new SetRightIntakeDeployed(false),
            new SetLeftIntakeDeployed(true),
            new SwerveTrajectoryCommand(ballAToBallB),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetRightIntakeDeployed(false),
            new ShootCargo(3.0)
    )),

    ROUTINE_FIVE_C("Routine_FIVE_B, move to pick up ball G, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(1.5),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetRightIntakeDeployed(true),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new SetRightIntakeDeployed(false),
            new SetLeftIntakeDeployed(true),
            new SwerveTrajectoryCommand(ballAToBallB),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetRightIntakeDeployed(false),
            new ShootCargo(3.0),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetLeftIntakeDeployed(true),
            new SwerveTrajectoryCommand(ballBToBallG),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetLeftIntakeDeployed(false),
            new SwerveTrajectoryCommand(ballGToBallB),
            new ShootCargo(3.0)
    )),

    ROUTINE_FIVE_D("ROUTINE_FIVE_B, move to pick up ball C, shoot 1", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(1.5),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetRightIntakeDeployed(true),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new SetRightIntakeDeployed(false),

            new SetLeftIntakeDeployed(true),
            new SwerveTrajectoryCommand(ballAToBallB),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetRightIntakeDeployed(false),
            new ShootCargo(3.0),

            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetLeftIntakeDeployed(true),
            new SwerveTrajectoryCommand(ballBToBallG),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetLeftIntakeDeployed(false),

            new SwerveTrajectoryCommand(ballGToBallB),
            new ShootCargo(3.0),

            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetLeftIntakeDeployed(true),
            new SwerveTrajectoryCommand(ballBToBallC),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetLeftIntakeDeployed(false),
            new ShootCargo(3.0)

    )),

    ROUTINE_FIVE_E("shoot from alpha, pick up ball A & ball B, come in to make 'money shot'", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(alphaPickUpA),
            new ShootCargo(1.5),

            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetRightIntakeDeployed(true),
            new SwerveTrajectoryCommand(alphaPickUpA),
            new SetRightIntakeDeployed(false),

            new SetLeftIntakeDeployed(true),
            new SwerveTrajectoryCommand(ballAToBallB),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetRightIntakeDeployed(false),

            new SwerveTrajectoryCommand(ballBToMoneyShot),
            new ShootCargo(3.0)
    )),

    ROUTINE_FIVE_B_FULL("ROUTINE_FIVE_B in one complete path", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(routineFiveBFull),
            new ShootCargo(1.5),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetLeftIntakeDeployed(true),
            new SetRightIntakeDeployed(true),
            new SwerveTrajectoryCommand(routineFiveBFull),
            new SetLeftIntakeDeployed(false),
            new SetRightIntakeDeployed(false),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new ShootCargo(3)
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

    ROUTINE_SIX_G("Start from gamma, pick up ball C, shoot 2, pick up ball G, move in to shoot", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaPickUpC),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetLeftIntakeDeployed(true),
            new SwerveTrajectoryCommand(gammaPickUpC),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetLeftIntakeDeployed(false),
            new SwerveTrajectoryCommand(ballCToShoot),
            new ShootCargo(3.0),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetRightIntakeDeployed(true),
            new SwerveTrajectoryCommand(ballCToBallG),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetRightIntakeDeployed(false),
            new SwerveTrajectoryCommand(ballGToBallB),
            new ShootCargo(3.0)
    )),

    ROUTINE_SEVEN("Start from gamma, pick up ball C, shoot 2, pick up ball B, pick up ball A, shoot 2", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaPickUpC),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new SetLeftIntakeDeployed(true),
            new SwerveTrajectoryCommand(gammaPickUpC),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetLeftIntakeDeployed(false),
            new SwerveTrajectoryCommand(ballCToShoot),
            new ShootCargo(3.0),
            new SwerveTrajectoryCommand(ballCToBallB),
            new SwerveTrajectoryCommand(ballBToBallA),
            new SwerveTrajectoryCommand(ballAToShoot),
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
