package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.*;

import static frc.robot.Constants.SmartDashboardKeys.kIntakeVoltageKey;
import static frc.robot.RobotContainer.controllerConfig;
import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.controller.XboxConfig.gamepad;
import static frc.robot.controller.XboxConfig.manipulationGamepad;

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
        if(manipulationGamepad.getRightTriggerAxis() > 0.75) {
            intake.setVoltage(SmartDashboard.getNumber(kIntakeVoltageKey, 9.5));
        } else if(controllerConfig.getOuttakeButton().get()){
            intake.setVoltage(-SmartDashboard.getNumber(kIntakeVoltageKey, 9.5));
        } else {
            intake.setVoltage(0.0);
        }
    }

    @Override
    public void end(boolean interrupted) {
        intake.setVoltage(0);
    }

}
