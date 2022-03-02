// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.util.WaltTimesliceRobot;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.VisionConstants.kAlignmentPipeline;
import static frc.robot.RobotContainer.godSubsystem;

/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */

public class Robot extends WaltTimesliceRobot {

    private Command autonomousCommand;

    private RobotContainer robotContainer;

    public Robot() {
        super(0.002, 0.007);
    }

    /**
     * This function is run when the robot is first started up and should be used for any
     * initialization code.
     */
    @Override
    public void robotInit() {
        // Instantiate our RobotContainer.  This will perform all our button bindings, and put our
        // autonomous chooser on the dashboard.
        robotContainer = new RobotContainer();

        // Schedule low-level periodic input and output methods for each subsystem
        // This is done to reduce CAN bus utilization
        schedule(godSubsystem.getDrivetrain()::collectData, 0.0003);
        schedule(godSubsystem.getDrivetrain()::outputData, 0.0006);
        schedule(godSubsystem.getIntake()::collectData, 0.0003);
        schedule(godSubsystem.getIntake()::outputData, 0.0006);
        schedule(godSubsystem.getConveyor()::collectData, 0.0003);
        schedule(godSubsystem.getConveyor()::outputData, 0.0006);
        schedule(godSubsystem.getShooter()::collectData, 0.0003);
        schedule(godSubsystem.getShooter()::outputData, 0.0006);
        schedule(godSubsystem.getClimber()::collectData, 0.0003);
//        schedule(godSubsystem.getClimber()::outputData, 0.0006);

        LimelightHelper.setLEDMode(kIsInTuningMode);
    }

    /**
     * This function is called every robot packet, no matter the mode. Use this for items like
     * diagnostics that you want ran during disabled, autonomous, teleoperated and test.
     *
     * <p>This runs after the mode specific periodic functions, but before LiveWindow and
     * SmartDashboard integrated updating.
     */
    @Override
    public void robotPeriodic() {
        // Runs the Scheduler.  This is responsible for polling buttons, adding newly-scheduled
        // commands, running already-scheduled commands, removing finished or interrupted commands,
        // and running subsystem periodic() methods.  This must be called from the robot's periodic
        // block in order for anything in the Command-based framework to work.
        CommandScheduler.getInstance().run();

//    SmartDashboard.putNumber("kX Position Error", kXController.getPositionError());
//    SmartDashboard.putNumber("kY Position Error", kYController.getPositionError());
//    SmartDashboard.putNumber("kTheta Position Error", kThetaController.getPositionError());
    }

    /**
     * This function is called once each time the robot enters Disabled mode.
     */
    @Override
    public void disabledInit() {
        godSubsystem.setEnabled(false);

        godSubsystem.setInAuton(false);
    }

    @Override
    public void disabledPeriodic() {
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        godSubsystem.setEnabled(true);

        // Clear auton flags
        godSubsystem.setInAuton(true);
        godSubsystem.setDoesAutonNeedToIntake(false);
        godSubsystem.setDoesAutonNeedToShoot(false);
        godSubsystem.setDoesAutonNeedToAlignAndShoot(false);

        godSubsystem.getDrivetrain().setCoastNeutralMode();

        LimelightHelper.setPipeline(kAlignmentPipeline);
        LimelightHelper.setLEDMode(kIsInTuningMode);

        autonomousCommand = robotContainer.getAutonomousCommand();

        // schedule the autonomous command (example)
        if (autonomousCommand != null) {
            autonomousCommand.schedule();
        }
    }

    /**
     * This function is called periodically during autonomous.
     */
    @Override
    public void autonomousPeriodic() {
    }

    @Override
    public void teleopInit() {
        godSubsystem.setEnabled(true);

        godSubsystem.setInAuton(false);

        godSubsystem.getDrivetrain().setCoastNeutralMode();

        LimelightHelper.setPipeline(kAlignmentPipeline);
        LimelightHelper.setLEDMode(true);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        godSubsystem.setEnabled(true);

        godSubsystem.setInAuton(false);

        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
    }
}
