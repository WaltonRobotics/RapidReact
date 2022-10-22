package frc.robot.commands.auton;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.Paths;
import frc.robot.util.UtilMethods;

import static frc.robot.Constants.Shooter.kShootingToleranceRawUnits;
import static frc.robot.Paths.FiveBallRoutine.*;
import static frc.robot.Paths.RoutineOne.gammaBackwards;
import static frc.robot.Paths.RoutineTwo.betaBackwards;
import static frc.robot.Paths.TestTrajectories.fiveFeetForward;
import static frc.robot.Paths.ThreeBallPickUpTwo.*;
import static frc.robot.Paths.TwoBall.gammaPickUpC;
import static frc.robot.Paths.TwoBallThrowRoutine.twoBallThrow;
import static frc.robot.RobotContainer.godSubsystem;

public enum AutonRoutine {
    DO_NOTHING("Do Nothing", new SequentialCommandGroup(
    )),

    FIVE_FEET_FORWARD("Moves forward five feet", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(fiveFeetForward),
            new SwerveTrajectoryCommand(fiveFeetForward)
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

    TWO_BALL("Start from gamma, pick up ball C, shoot 2", new TimedAuton(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaPickUpC),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true)),
            new SetLeftIntakeDeployed(true),
            new ParallelCommandGroup(
                    new SwerveTrajectoryCommand(gammaPickUpC),
                    new SequentialCommandGroup(
                            new WaitCommand(0.15),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true))
                    )
            ),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            new SetLeftIntakeDeployed(false),
//            new TurnToAngle(90.0).withTimeout(2.0),
            new WaitCommand(0.7),
            new AlignAndShootCargoTimed(10.0)
    )),

    TWO_BALL_THROW("Start from gamma, pick up ballC, shoot 2, throw enemy ball into hanger", new TimedAuton(
            //normal two ball
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(gammaPickUpC),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true)),
            new SetLeftIntakeDeployed(true),
            new ParallelCommandGroup(
                    new SwerveTrajectoryCommand(gammaPickUpC),
                    new SequentialCommandGroup(
                            new WaitCommand(gammaPickUpC.getTotalTimeSeconds() * 0.75),
                            new SetLeftIntakeDeployed(false)
                    ),
                    new SequentialCommandGroup(
                            new WaitCommand(0.15),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true))
                    )
            ),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
            //wait for 5-ball robots
            new WaitCommand(1.25),
            new ParallelDeadlineGroup(
                    new ShootCargo(2, 3.0),
                    new SequentialCommandGroup(
                            new WaitCommand(0.1),
                            new WaitUntilCommand(() ->
                                    UtilMethods.isWithinTolerance(godSubsystem.getShooter().getFlywheelVelocityNU(),
                                            godSubsystem.getShooter().getFlywheelDemand(),kShootingToleranceRawUnits)),
                            new SetLeftIntakeDeployed(true)
                    )
            ),
            //throw enemy ball
            new ParallelDeadlineGroup(
                    new SwerveTrajectoryCommand(twoBallThrow),
                    new ParallelCommandGroup(
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
                            new SetLeftIntakeDeployed(true),
                            new WaitCommand(twoBallThrow.getTotalTimeSeconds() * 0.5),  //spin up 50% through path
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true))
                    )
            ),
            new TurnToAngle(190).withTimeout(2.0),
            new ParallelCommandGroup(
                    new BarfBall(1, 5.0),
                    new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false)),
                    new SetLeftIntakeDeployed(false)
            ),
            new TurnToAngle(-90).withTimeout(2.0)
    )),

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

    FIVE_BALL("Pick up & shoot 2, shoot 1, pick up & shoot 2", new SequentialCommandGroup(
            new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
            new ResetPose(fiveBall1),
            new SetRightIntakeDeployed(true),
            new SetLeftIntakeDeployed(true),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(true)),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(true)),
            new SwerveTrajectoryCommand(fiveBall1),
            new SetRightIntakeDeployed(false),
            new ParallelDeadlineGroup(
                    new ShootCargo(3, 1.2),
                    new SequentialCommandGroup(
                            new WaitCommand(.75),
                            new SetLeftIntakeDeployed(false)
                    )
            ),
            new ParallelCommandGroup(
                    new SwerveTrajectoryCommand(fiveBall2),
                    new SequentialCommandGroup(
                            new WaitCommand(fiveBall2.getTotalTimeSeconds() * 0.4),
                            new SetLeftIntakeDeployed(true)
                    )
            ),
            new WaitCommand(.25),
            new ParallelDeadlineGroup(
                    new SwerveTrajectoryCommand(fiveBall3),
                    new SequentialCommandGroup(
                            new WaitCommand(fiveBall3.getTotalTimeSeconds() * 0.75),
                            new SetLeftIntakeDeployed(false),
                            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIntake(false))
                    )
            ),
            new AlignAndShootCargo(2, 4.0),
            new InstantCommand(() -> godSubsystem.setDoesAutonNeedToIdleSpinUp(false))
    )),

    STRAIGHT_FORWARD("Straight", new SequentialCommandGroup(
            new Straight(Units.feetToMeters(6.0))
    )),

    STRAIGHT_BACKWARD("Straight", new SequentialCommandGroup(
            new Straight(-Units.feetToMeters(6.0))
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
