// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.SuperstructureCommand;
import frc.robot.commands.auton.RotateModulesToAngle;
import frc.robot.controller.ControllerConfig;
import frc.robot.controller.GamepadsConfig;
import frc.robot.robots.RobotIdentifier;
import frc.robot.robots.WaltRobot;
import frc.robot.subsystems.Superstructure;

import java.util.logging.Level;
import java.util.logging.Logger;

import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DioIDs.kRobotID1;
import static frc.robot.Constants.DioIDs.kRobotID2;
import static frc.robot.Constants.SmartDashboardKeys.*;

import frc.robot.vision.ColorSensor;
import frc.robot.vision.ColorSensor.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  public static final WaltRobot currentRobot;
  public static final Superstructure godSubsystem;
  public static final ControllerConfig controllerConfig;
  public static final Logger robotLogger = Logger.getLogger("frc.robot");

  static {
    currentRobot = RobotIdentifier.findByInputs(new DigitalInput(kRobotID1).get(),
            new DigitalInput(kRobotID2).get()).getSelectedRobot();

    godSubsystem = new Superstructure();

    controllerConfig = new GamepadsConfig();

    robotLogger.setLevel(Level.FINEST);

    //colorSensor = new ColorSensorV3(i2cPort);
  }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    CommandScheduler.getInstance().setDefaultCommand(godSubsystem, new SuperstructureCommand());
    CommandScheduler.getInstance().setDefaultCommand(godSubsystem.getDrivetrain(), new DriveCommand());

    initShuffleboard();

    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {}

  private void initShuffleboard() {
    SmartDashboard.putData(kDrivetrainRotateModulesToAngleKey, new RotateModulesToAngle());

    SmartDashboard.putNumber(kDrivetrainSetpointAngleDegreesKey, 0.0);

    if (kIsInTuningMode) {
      SmartDashboard.putNumber(kDrivetrainLeftFrontZeroValueKey, 0.0);
      SmartDashboard.putNumber(kDrivetrainRightFrontZeroValueKey, 0.0);
      SmartDashboard.putNumber(kDrivetrainLeftRearZeroValueKey, 0.0);
      SmartDashboard.putNumber(kDrivetrainRightRearZeroValueKey, 0.0);

      SmartDashboard.putData(kDrivetrainSaveLeftFrontZeroKey,
              new InstantCommand(() ->
                      godSubsystem.getDrivetrain().saveLeftFrontZero((int)SmartDashboard.getNumber(kDrivetrainLeftFrontZeroValueKey, 0.0))));

      SmartDashboard.putData(kDrivetrainSaveRightFrontZeroKey,
              new InstantCommand(() ->
                      godSubsystem.getDrivetrain().saveRightFrontZero((int)SmartDashboard.getNumber(kDrivetrainRightFrontZeroValueKey, 0.0))));

      SmartDashboard.putData(kDrivetrainSaveLeftRearZeroKey,
              new InstantCommand(() ->
                      godSubsystem.getDrivetrain().saveLeftRearZero((int)SmartDashboard.getNumber(kDrivetrainLeftRearZeroValueKey, 0.0))));

      SmartDashboard.putData(kDrivetrainSaveRightRearZeroKey,
              new InstantCommand(() ->
                      godSubsystem.getDrivetrain().saveRightRearZero((int)SmartDashboard.getNumber(kDrivetrainRightRearZeroValueKey, 0.0))));

      SmartDashboard.putData(kDriverForwardScale, controllerConfig.getForwardScale());
      SmartDashboard.putData(kDriverStrafeScale, controllerConfig.getStrafeScale());
      SmartDashboard.putData(kDriverYawScale, controllerConfig.getYawScale());


    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    return new InstantCommand();
  }
}
