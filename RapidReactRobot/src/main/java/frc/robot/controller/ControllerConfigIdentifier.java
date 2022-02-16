package frc.robot.controller;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;
import java.util.Map;
import java.util.function.Supplier;

public class ControllerConfigIdentifier {

    static Map<String[], Supplier<ControllerConfig>> configMap = new HashMap<>();

    static {
        configMap.put(new String[]{"Xbox Controller", "Xbox Controller"}, XboxConfig::new);
    }

    public static ControllerConfig getInferredControllerConfig() {
        for(Map.Entry<String[], Supplier<ControllerConfig>> entry : configMap.entrySet() ){
            String[] devices = entry.getKey();

            boolean matches = true;

            for(int i = 0; i < devices.length; i++) {
                if(!DriverStation.getJoystickName(i).equals(devices[i])){
                    matches = false;
                    break;
                }
            }

            if(matches) {
                Supplier<ControllerConfig> configSupplier = entry.getValue();
                return configSupplier.get();
            }
        }

        return new XboxConfig();
    }

}
