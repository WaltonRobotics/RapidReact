package frc.robot.config;

import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;

public interface ClimberConfig {

    TalonFXConfiguration getPivotControllerConfig();
    TalonFXConfiguration getExtensionControllerConfig();


}
