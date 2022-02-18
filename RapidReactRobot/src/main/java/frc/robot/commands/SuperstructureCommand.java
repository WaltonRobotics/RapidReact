package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.OI;
import frc.robot.stateMachine.IState;
import frc.robot.stateMachine.StateMachine;
import frc.robot.subsystems.*;


import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.subsystems.Intake.IntakeControlState.DISABLED;
import static frc.robot.subsystems.Intake.IntakeControlState.VOLTAGE;

public class SuperstructureCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Climber climber;
    private StateMachine stateMachine;

    private IState disabled;
    private IState initializing;
    private IState idle;
    private IState intaking;
    private IState outtaking;
    private IState AligningAndSpinningUp;
    private IState shooting;
    private IState spinningUp;
    private IState adjustingHood;


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
        stateMachine = new StateMachine("disabled", disabled);

        disabled = new IState(){

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
                return null;
            }
        };

        idle = new IState(){

            @Override
            public void initialize() {
                //TODO: set safe subsystem limits
                intake.setIntakeControlState(Intake.IntakeControlState.DISABLED);
                shooter.setShooterControlState(Shooter.ShooterControlState.DISABLED);
                climber.setPivotControlState(Climber.ClimberControlState.DISABLED);
                climber.setExtensionControlState(Climber.ClimberControlState.DISABLED);
                conveyor.setConveyorControlState(Conveyor.ConveyorControlState.DISABLED);
            }

            @Override
            public IState execute() {
                if(OI.intakeButton.getAsBoolean()){
                    return intaking;
                }

                if(OI.outtakeButton.getAsBoolean()){
                    return outtaking;
                }

                return this;
                //TODO: handle manual overrides


            }



            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return null;
            }
        };

        intaking = new IState(){
            @Override
            public void initialize() {
                conveyor.setConveyorControlState(Conveyor.ConveyorControlState.VOLTAGE);
                intake.setIntakeControlState(Intake.IntakeControlState.VOLTAGE);
                intake.setLeftIntakeDeployStateDemand(true);
                intake.setRightIntakeDeployStateDemand(true);

            }

            @Override
            public IState execute() {
                //dummy voltage values for the following:
                conveyor.setFeedDemand(8.0);
                conveyor.setTransportDemand(8.0);
                intake.setLeftIntakeDemand(8.0);
                intake.setRightIntakeDemand(8.0);
                return null;
            }

            @Override
            public void finish() {

            }

            @Override
            public String getName() {
                return null;
            }
        };

        outtaking = new IState(){

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
                return null;
            }
        };

        shooting = new IState(){

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
                return null;
            }
        };

        adjustingHood = new IState(){

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
                return null;
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
