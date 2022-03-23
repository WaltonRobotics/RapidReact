// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.auton.AutonRoutine;
import frc.robot.commands.auton.SetModuleStates;
import frc.robot.robots.RobotIdentifier;
import frc.robot.robots.WaltRobot;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Superstructure;
import frc.robot.vision.LimelightHelper;

import java.util.Arrays;
import java.util.logging.Level;
import java.util.logging.Logger;

import static frc.robot.Constants.ContextFlags.kIsInShooterTuningMode;
import static frc.robot.Constants.ContextFlags.kIsInTuningMode;
import static frc.robot.Constants.DioIDs.kRobotID1;
import static frc.robot.Constants.DioIDs.kRobotID2;
import static frc.robot.Constants.Shooter.kDefaultVelocityRawUnits;
import static frc.robot.Constants.SmartDashboardKeys.*;
import static frc.robot.OI.*;
import static frc.robot.commands.auton.AutonRoutine.DO_NOTHING;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    public static final WaltRobot currentRobot;
    public static final Superstructure godSubsystem;
    public static final Logger robotLogger = Logger.getLogger("frc.robot");
    public static final SendableChooser<AutonRoutine> autonChooser = new SendableChooser<>();

    static {
        currentRobot = RobotIdentifier.findByInputs(new DigitalInput(kRobotID1).get(),
                new DigitalInput(kRobotID2).get()).getSelectedRobot();

        robotLogger.log(Level.INFO, "Current robot: " + currentRobot.getClass().getSimpleName());

        godSubsystem = new Superstructure();

        robotLogger.setLevel(Level.FINEST);
    }

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        CommandScheduler.getInstance().registerSubsystem(godSubsystem);
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
    private void configureButtonBindings() {
        resetDrivetrainButton.whenPressed(new SequentialCommandGroup(
                new InstantCommand(() -> godSubsystem.getDrivetrain().zeroSensors()),
                new InstantCommand(() -> System.out.println("Reset drivetrain"))
        ));

        toggleLimelightButton.whenPressed(LimelightHelper::toggleLimelight);

        // Manual overrides
        toggleLeftIntakeButton.whenPressed(new InstantCommand(
                () -> {
                    if (godSubsystem.getCurrentMode() == Superstructure.CurrentMode.SCORING_MODE) {
                        godSubsystem.getIntake().toggleLeftIntakeDeployStateDemand();
                    }
                }
        ));

        toggleRightIntakeButton.whenPressed(new InstantCommand(
                () -> {
                    if (godSubsystem.getCurrentMode() == Superstructure.CurrentMode.SCORING_MODE) {
                        godSubsystem.getIntake().toggleRightIntakeDeployStateDemand();
                    }
                }
        ));

        toggleClimberLocksButton.whenPressed(godSubsystem.getClimber()::toggleClimberLock);

        aimUpperButton.whenPressed(() -> godSubsystem.getShooter().setAimTarget(Shooter.AimTarget.HIGH_GOAL));
        aimLowerButton.whenPressed(() -> godSubsystem.getShooter().setAimTarget(Shooter.AimTarget.LOW_GOAL));
    }

    public void initShuffleboard() {
        LiveWindow.disableAllTelemetry();

        SmartDashboard.putData(kDrivetrainSetModuleStatesKey, new SetModuleStates());

        SmartDashboard.putData(kDrivetrainResetKey, new InstantCommand(godSubsystem.getDrivetrain()::zeroSensors));

        SmartDashboard.putNumber(kDrivetrainSetpointAngleDegreesKey, 0.0);
        SmartDashboard.putNumber(kDrivetrainSetpointVelocityKey, 0.0);

        SmartDashboard.putNumber(kDrivetrainHeadingDegrees, 0.0);
        SmartDashboard.putNumber(kDrivetrainAngularVelocity, 0.0);
        SmartDashboard.putNumber(kDrivetrainPitchDegrees, 0.0);
        SmartDashboard.putNumber(kDrivetrainRollDegrees, 0.0);

        SmartDashboard.putBoolean(kDrivetrainIsFieldRelativeKey, true);
        SmartDashboard.putBoolean(kDrivetrainIsPositionalRotationKey, false);

        SmartDashboard.putNumber(kClimberPivotAngleFromVerticalKey, 0.0);
        SmartDashboard.putNumber(kClimberPivotAngleFromHorizontalKey, 0.0);

        SmartDashboard.putNumber(kShooterCurrentTargetVelocityKey, 0.0);

        SmartDashboard.putNumber(kShooterBallQualityAdditive, 0.0);

        SmartDashboard.putNumber(kTrajectoryThetaPKey,
                godSubsystem.getDrivetrain().getConfig().getThetaController().getP());

        // Auton chooser
        Arrays.stream(AutonRoutine.values()).forEach(n -> autonChooser.addOption(n.name(), n));
        autonChooser.setDefaultOption(DO_NOTHING.name(), DO_NOTHING);
        SmartDashboard.putData("Auton Selector", autonChooser);

        SmartDashboard.putBoolean(kDriverIsAlignedKey, false);
        SmartDashboard.putBoolean(kDriverIsMoneyShotKey, false);

        SmartDashboard.putString(kDriverSelectedRungKey, godSubsystem.getSelectedRung().name());

        SmartDashboard.putBoolean(kClimberPivotCoastModeKey, false);
        SmartDashboard.putBoolean(kClimberExtensionCoastModeKey, false);

        if (kIsInTuningMode) {
            SmartDashboard.putNumber(kDrivetrainLeftFrontZeroValueKey, 0.0);
            SmartDashboard.putNumber(kDrivetrainRightFrontZeroValueKey, 0.0);
            SmartDashboard.putNumber(kDrivetrainLeftRearZeroValueKey, 0.0);
            SmartDashboard.putNumber(kDrivetrainRightRearZeroValueKey, 0.0);

            SmartDashboard.putData(kDrivetrainSaveLeftFrontZeroKey,
                    new InstantCommand(() ->
                            godSubsystem.getDrivetrain().saveLeftFrontZero((int) SmartDashboard.getNumber(kDrivetrainLeftFrontZeroValueKey, 0.0))));

            SmartDashboard.putData(kDrivetrainSaveRightFrontZeroKey,
                    new InstantCommand(() ->
                            godSubsystem.getDrivetrain().saveRightFrontZero((int) SmartDashboard.getNumber(kDrivetrainRightFrontZeroValueKey, 0.0))));

            SmartDashboard.putData(kDrivetrainSaveLeftRearZeroKey,
                    new InstantCommand(() ->
                            godSubsystem.getDrivetrain().saveLeftRearZero((int) SmartDashboard.getNumber(kDrivetrainLeftRearZeroValueKey, 0.0))));

            SmartDashboard.putData(kDrivetrainSaveRightRearZeroKey,
                    new InstantCommand(() ->
                            godSubsystem.getDrivetrain().saveRightRearZero((int) SmartDashboard.getNumber(kDrivetrainRightRearZeroValueKey, 0.0))));

//            SmartDashboard.putData(kDriverForwardScaleKey, OI.forwardScale);
//            SmartDashboard.putData(kDriverStrafeScaleKey, OI.strafeScale);
//            SmartDashboard.putData(kDriverYawScaleKey, OI.yawScale);

            SmartDashboard.putData("kXController", currentRobot.getDrivetrainConfig().getXController());
            SmartDashboard.putData("kYController", currentRobot.getDrivetrainConfig().getYController());
            SmartDashboard.putData("kThetaController", currentRobot.getDrivetrainConfig().getThetaController());

            SmartDashboard.putNumber("X Error Average", 0.0);
            SmartDashboard.putNumber("Y Error Average", 0.0);
            SmartDashboard.putNumber("Theta Error Average", 0.0);

            SmartDashboard.putData(godSubsystem.getDrivetrain().getConfig().getFaceDirectionController());

            SmartDashboard.putData(kLimelightAlignControllerKey, currentRobot.getDrivetrainConfig().getAutoAlignController());
            SmartDashboard.putNumber(kLimelightAlignErrorDegrees, 0.0);
            SmartDashboard.putNumber(kLimelightAlignOmegaOutputKey, 0.0);
            SmartDashboard.putNumber(kLimelightDistanceFeetKey, 0.0);

            SmartDashboard.putData(kTurnToAngleControllerKey, currentRobot.getDrivetrainConfig().getTurnToAngleController());
            SmartDashboard.putNumber(kTurnToAngleErrorDegreesKey, 0.0);
            SmartDashboard.putNumber(kTurnToAngleOmegaOutputKey, 0.0);

            SmartDashboard.putNumber(kLeftIntakePercentOutputKey, godSubsystem.getIntake().getConfig().getLeftIntakePercentOutput());
            SmartDashboard.putNumber(kRightIntakePercentOutputKey, godSubsystem.getIntake().getConfig().getRightIntakePercentOutput());
        }

        if (kIsInShooterTuningMode) {
            SmartDashboard.putNumber(kShooterTuningSetpointVelocityNUKey, kDefaultVelocityRawUnits);

            SmartDashboard.putNumber(kShooterHoodPositionSetpointKey, 0.0);

            SmartDashboard.putData("Move Half Foot Backwards",
                    AutonRoutine.HALF_FOOT_BACKWARDS.getCommandGroup());
        }

        SmartDashboard.putBoolean("Flywheel on target", false);

        NetworkTableInstance.getDefault().flush();
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        AutonRoutine routine = autonChooser.getSelected();

        robotLogger.log(Level.INFO, "Selected autonomous description: " + routine.getDescription());

        return routine.getCommandGroup();
    }

}
