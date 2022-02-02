package frc.robot.commands;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Subsystem;

import static frc.robot.RobotContainer.godSubsystem;
import static frc.robot.Robot.*;

public class IntakeCommand extends CommandBase {

    public IntakeCommand(){
        addRequirements((Subsystem) godSubsystem.getIntake());


    }

    @Override
    public void execute(){
//        if(intakeButton.get()) {
//
//
//            godSubsystem.getIntake().setVoltage(SmartDashboard.getNumber("Intake voltage", 8));
//        }
//        else if(outtakeButton.get()){
//            godSubsystem.getIntake().setVoltage(-SmartDashboard.getNumber("Intake voltage", 8));
//        }
//        else{
//            godSubsystem.getIntake().setVoltage(0.0);
//        }

//        intake.setVoltage(8.0);
    }

    @Override
    public void end(boolean interrupted){
        godSubsystem.getIntake().setVoltage(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}