package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.*;
import frc.robot.vision.LimelightHelper;

import static frc.robot.RobotContainer.godSubsystem;

public class SuperstructureCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Climber climber;
    private final StateMachine stateMachine;

    private IState disabled;
    private IState initializing;
    private final IState idle;
    private final IState intaking;
    private final IState outtaking;
    private IState AligningAndSpinningUp;
    private final IState shooting;
    private IState spinningUp;
    private final IState adjustingHood;


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
        stateMachine = new StateMachine("Disabled", disabled);

        disabled = new IState() {

            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                return null;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Disabled";
            }
        };

        idle = new IState() {

            @Override
            public void initialize() {
                //TODO: set safe subsystem limits
                //set subsystem control modes to disabled
                intake.setIntakeControlState(Intake.IntakeControlState.DISABLED);
                shooter.setShooterControlState(Shooter.ShooterControlState.DISABLED);
                climber.setPivotControlState(Climber.ClimberControlState.DISABLED);
                climber.setExtensionControlState(Climber.ClimberControlState.DISABLED);
                conveyor.setConveyorControlState(Conveyor.ConveyorControlState.DISABLED);
            }

            @Override
            public IState execute() {
                if (OI.intakeButton.getAsBoolean()) {
                    return intaking;
                }

                if (OI.outtakeButton.getAsBoolean()) {
                    return outtaking;
                }

                //manual overrides
                if (OI.toggleLeftIntakeButton.getAsBoolean()) {
                    intake.setLeftIntakeDeployStateDemand(true);
                }
                if (OI.toggleRightIntakeButton.getAsBoolean()) {
                    intake.setRightIntakeDeployStateDemand(true);
                }

                if (OI.overrideTransportConveyorButton.getAsBoolean()) {
                    conveyor.setTransportDemand(8.0);   //dummy voltage value
                } else {
                    conveyor.setTransportDemand(0);
                }

                if (OI.overrideFeedConveyorButton.getAsBoolean()) {
                    conveyor.setFeedDemand(8.0);    //dummy voltage vale
                } else {
                    conveyor.setFeedDemand(0.0);
                }
                //TODO: climber manual overrides

                return this;
            }


            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Idle";
            }
        };

        intaking = new IState() {
            @Override
            public void initialize() {
                conveyor.setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
                intake.setIntakeControlState(Intake.IntakeControlState.VOLTAGE);
            }

            @Override
            public IState execute() {
                //dummy voltage values for the following:
                if (intake.isLeftIntakeDeployStateDemand()) {
                    intake.setLeftIntakeDemand(8.0);
                }

                if (intake.isRightIntakeDeployStateDemand()) {
                    intake.setRightIntakeDemand(8.0);
                }
                conveyor.setFeedDemand(8.0);
                conveyor.setTransportDemand(8.0);
                return idle;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Intaking";
            }
        };

        outtaking = new IState() {

            @Override
            public void initialize() {
                conveyor.setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
                intake.setIntakeControlState(Intake.IntakeControlState.VOLTAGE);
            }

            @Override
            public IState execute() {
                //dummy voltage values for the following:
                if (intake.isLeftIntakeDeployStateDemand()) {
                    intake.setLeftIntakeDemand(-8.0);
                }

                if (intake.isRightIntakeDeployStateDemand()) {
                    intake.setRightIntakeDemand(-8.0);
                }
                conveyor.setFeedDemand(-8.0);
                conveyor.setTransportDemand(-8.0);
                return idle;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return null;
            }
        };

        adjustingHood = new IState() {

            @Override
            public void initialize() {
                if (LimelightHelper.getDistanceToTargetMeters() <= 5) { //dummy value
                    shooter.setRightAdjustableHoodDutyCycleDemand(10);
                    shooter.setLeftAdjustableHoodDutyCycleDemand(10);
                } else {
                    shooter.setRightAdjustableHoodDutyCycleDemand(20);
                    shooter.setLeftAdjustableHoodDutyCycleDemand(20);
                }
            }

            @Override
            public IState execute() {
                //the following are dummy values
                if (!OI.shootButtonButton.getAsBoolean()) {
                    return idle;
                }
                return shooting;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Adjusting Hood";
            }
        };

        shooting = new IState() {

            @Override
            public void initialize() {

            }

            @Override
            public IState execute() {
                return null;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return "Shooting";
            }
        };

    }

    @Override
    public void execute() {
        stateMachine.run();

    }

    @Override
    public void end(boolean interrupted) {

    }

}
