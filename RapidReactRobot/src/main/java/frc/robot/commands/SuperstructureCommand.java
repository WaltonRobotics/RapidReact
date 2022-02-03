package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

import static frc.robot.Constants.SmartDashboardKeys.kIntakeVoltage;
import static frc.robot.RobotContainer.controllerConfig;
import static frc.robot.RobotContainer.godSubsystem;

public class SuperstructureCommand extends CommandBase {

    private final Drivetrain drivetrain;
    private final Intake intake;
    private final Conveyor conveyor;
    private final Shooter shooter;
    private final Climber climber;

    public SuperstructureCommand() {
        addRequirements(godSubsystem);

        drivetrain = godSubsystem.getDrivetrain();
        intake = godSubsystem.getIntake();
        conveyor = godSubsystem.getConveyor();
        shooter = godSubsystem.getShooter();
        climber = godSubsystem.getClimber();
    }

    @Override
    public void execute() {
        if(controllerConfig.getIntakeButton().get()) {
            godSubsystem.getIntake().setVoltage(SmartDashboard.getNumber(kIntakeVoltage, 9.5));
        }
        else if(controllerConfig.getOuttakeButton().get()){
            godSubsystem.getIntake().setVoltage(-SmartDashboard.getNumber(kIntakeVoltage, 9.5));
        }
        else{
            godSubsystem.getIntake().setVoltage(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        godSubsystem.getIntake().setVoltage(0);
    }

}
