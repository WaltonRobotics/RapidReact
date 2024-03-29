// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.util.net.PortForwarder;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import frc.lib.strykeforce.swerve.SwerveModule;
import frc.robot.commands.auton.SetModuleStates;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.util.WaltTimesliceRobot;
import frc.robot.vision.LimelightHelper;

import static frc.robot.Constants.Climber.kPivotArmNudgeIncrementNU;
import static frc.robot.Constants.ContextFlags.*;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.Constants.VisionConstants.kAlignmentPipeline;
import static frc.robot.OI.driveGamepad;
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

    private final PowerDistribution pdp = new PowerDistribution();
    private final PneumaticHub pneumaticHub = new PneumaticHub();

    private boolean pitCheckConfigureClimber = false;
    private double pitCheckStartTime;

    private boolean hasSeenFMS = false;

    public Robot() {
        super(0.002, 0.02);
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
        schedule(godSubsystem.getClimber()::outputData, 0.0006);

        LimelightHelper.setLEDMode(kIsInTuningMode);

        PortForwarder.add(5800, "10.29.74.11", 5800);
        PortForwarder.add(5801, "10.29.74.11", 5801);
        PortForwarder.add(5802, "10.29.74.11", 5802);
        PortForwarder.add(5803, "10.29.74.11", 5803);
        PortForwarder.add(5804, "10.29.74.11", 5804);
        PortForwarder.add(5805, "10.29.74.11", 5805);

        // Schedule updating shuffleboard on a separate thread with lower frequency
        // to prevent network latency
        addPeriodic(godSubsystem::updateShuffleboard, 0.25);

        // Monitor motor temperatures every second
        addPeriodic(godSubsystem::monitorTemperatures, 1.0);
//        addPeriodic(godSubsystem::handleLEDLights, .25);


        LimelightHelper.setLEDMode(false);
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
        godSubsystem.getClimber().setClimberLockStateDemand(false);
        godSubsystem.getClimber().setHighBarArmsDeployed(false);

        godSubsystem.setEnabled(false);

        godSubsystem.setInAuton(false);

        godSubsystem.setIsInPitCheckMode(false);

        SmartDashboard.putBoolean(kClimberPivotCoastModeKey, false);
        SmartDashboard.putBoolean(kClimberExtensionCoastModeKey, false);

//        LimelightHelper.setLEDMode(false);

        godSubsystem.getDrivetrain().setCoastNeutralMode();

        SmartDashboard.putData(kDrivetrainSetModuleStatesKey, new SequentialCommandGroup(
                new SetModuleStates()
        ));

        SmartDashboard.putData(kDrivetrainResetKey, new InstantCommand(godSubsystem.getDrivetrain()::zeroSensors));

        SmartDashboard.putNumber(kDrivetrainSetpointAngleDegreesKey, 0.0);
        SmartDashboard.putNumber(kDrivetrainSetpointVelocityKey, 0.0);
    }

    @Override
    public void disabledPeriodic() {
        if (SmartDashboard.getBoolean(kClimberPivotCoastModeKey, false)) {
            godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Coast);
        } else {
            godSubsystem.getClimber().setPivotNeutralMode(NeutralMode.Brake);
        }

        if (SmartDashboard.getBoolean(kClimberExtensionCoastModeKey, false)) {
            godSubsystem.getClimber().setExtensionNeutralMode(NeutralMode.Coast);
        } else {
            godSubsystem.getClimber().setExtensionNeutralMode(NeutralMode.Brake);
        }

        if(DriverStation.isFMSAttached() && !hasSeenFMS){
            kIsInShooterTuningMode = false;
            kIsInCompetition = true;
            kIsInTuningMode = true;
        }
    }

    /**
     * This autonomous runs the autonomous command selected by your {@link RobotContainer} class.
     */
    @Override
    public void autonomousInit() {
        // Clear faults before a match for easy diagnosis of faults during the match
        pdp.clearStickyFaults();
        pneumaticHub.clearStickyFaults();

        godSubsystem.setEnabled(true);

        // Clear auton flags
        godSubsystem.setInAuton(true);
        godSubsystem.setDoesAutonNeedToIntake(false);
        godSubsystem.setDoesAutonNeedToShoot(false);
        godSubsystem.setDoesAutonNeedToAlignAndShoot(false);
        godSubsystem.setDoesAutonNeedToBarf(false);

        godSubsystem.setIsInPitCheckMode(false);

        SmartDashboard.putBoolean(kClimberPivotCoastModeKey, false);
        SmartDashboard.putBoolean(kClimberExtensionCoastModeKey, false);

        if (kIsInShooterTuningMode) {
            godSubsystem.getDrivetrain().setCoastNeutralMode();
        } else {
            godSubsystem.getDrivetrain().setBrakeNeutralMode();
        }

        // Config follower due to bug in testing mode
        godSubsystem.getShooter().configFollower();

        godSubsystem.getShooter().setAimTarget(Shooter.AimTarget.HIGH_GOAL);

        LimelightHelper.setPipeline(kAlignmentPipeline);
        LimelightHelper.setLEDMode(true);

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
        godSubsystem.setCurrentMode(Superstructure.CurrentMode.SCORING_MODE);

        godSubsystem.setEnabled(true);

        godSubsystem.setInAuton(false);
        godSubsystem.setIsInPitCheckMode(false);

        SmartDashboard.putBoolean(kClimberPivotCoastModeKey, false);
        SmartDashboard.putBoolean(kClimberExtensionCoastModeKey, false);

        if (kIsInShooterTuningMode) {
            godSubsystem.getDrivetrain().setCoastNeutralMode();
        } else {
            godSubsystem.getDrivetrain().setBrakeNeutralMode();
        }

        // Config follower due to bug in testing mode
        godSubsystem.getShooter().configFollower();

        LimelightHelper.setPipeline(kAlignmentPipeline);
        LimelightHelper.setLEDMode(true);

        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) {
            autonomousCommand.cancel();
        }

        new RunCommand(() -> {
            driveGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0.25);
            driveGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0.25);
        }).withTimeout(2.5).andThen(() -> {
            driveGamepad.setRumble(GenericHID.RumbleType.kLeftRumble, 0);
            driveGamepad.setRumble(GenericHID.RumbleType.kRightRumble, 0);
        }).schedule();
    }

    /**
     * This function is called periodically during operator control.
     */
    @Override
    public void teleopPeriodic() {
    }

    @Override
    public void testInit() {
        godSubsystem.setEnabled(false);

        godSubsystem.setInAuton(false);
        godSubsystem.setIsInPitCheckMode(true);

        pitCheckConfigureClimber = true;
        pitCheckStartTime = Timer.getFPGATimestamp();

        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();

        godSubsystem.getClimber().zeroSensors();

//        godSubsystem.getClimber().setClimberLockStateDemand(true);

        SmartDashboard.putData("Nudge Climber", new InstantCommand(() -> {
            if (!godSubsystem.isEnabled() && godSubsystem.isInPitCheckMode()) {
                double currentPivotAngle = godSubsystem.getClimber().getPivotPositionDemandNU();

                godSubsystem.getClimber().setPivotPositionDemandNU(currentPivotAngle + kPivotArmNudgeIncrementNU);
            }
        }));

        CommandScheduler.getInstance().enable();
    }

    /**
     * This function is called periodically during test mode.
     */
    @Override
    public void testPeriodic() {
        if (pitCheckConfigureClimber && Timer.getFPGATimestamp() - pitCheckStartTime > 0.25) {
            godSubsystem.getClimber().setPivotControlState(Climber.ClimberControlState.AUTO);
            godSubsystem.getClimber().setPivotPositionDemandNU(godSubsystem.getClimber().getPivotIntegratedEncoderPositionNU());
            godSubsystem.getClimber().setPivotLimits(Climber.ClimberPivotLimits.PIVOT_FULL_ROM);

            pitCheckConfigureClimber = false;
        }

//        for (SwerveModule module : godSubsystem.getDrivetrain().getSwerveModules()) {
//            module.setDesiredState(new SwerveModuleState(godSubsystem.getDrivetrain().getConfig().getMaxSpeedMetersPerSecond() * 0.6, Rotation2d.fromDegrees(SmartDashboard.getNumber(kDrivetrainSetpointVelocityKey, 0.0))), true);
//        }

        godSubsystem.getDrivetrain().setModuleStates(new SwerveModuleState(
                SmartDashboard.getNumber(kDrivetrainSetpointVelocityKey, 0.0),
                Rotation2d.fromDegrees(SmartDashboard.getNumber(kDrivetrainSetpointAngleDegreesKey, 0.0))
        ));
    }
}
