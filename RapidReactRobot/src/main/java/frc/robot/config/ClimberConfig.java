package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import edu.wpi.first.math.controller.ProfiledPIDController;
import frc.robot.subsystems.Climber;

import java.util.HashMap;

public interface ClimberConfig {

    HashMap<Climber.ClimberPivotLimits, LimitPair> getClimberPivotLimits();

    HashMap<Climber.ClimberPivotPosition, Target> getClimberPivotTargets();

    HashMap<Climber.ClimberExtensionLimits, LimitPair> getClimberExtensionLimits();

    HashMap<Climber.ClimberExtensionPosition, Target> getClimberExtensionTargets();

    double getVerticalReferenceAbsoluteCounts();

    double getMaxGravityFeedForward();

    MotorConfig getPivotControllerMotorConfig();

    MotorConfig getExtensionControllerMotorConfig();

    TalonFXConfiguration getPivotControllerTalonConfig();

    TalonFXConfiguration getExtensionControllerTalonConfig();

    AbsoluteEncoderConfig getPivotAngleAbsoluteEncoderConfig();

    int getLeftExtensionLowerLimitChannel();

    int getRightExtensionLowerLimitChannel();

    int getClimberLockSolenoidChannel();
    int getClimberDiscBrakeSolenoidChannel();

    double getPivotPercentOutputLimit();
    double getExtensionManualPercentOutputLimit();

    double getAbsoluteCountsToIntegratedCountsFactor();

    ProfiledPIDController getPivotProfiledController();

}
