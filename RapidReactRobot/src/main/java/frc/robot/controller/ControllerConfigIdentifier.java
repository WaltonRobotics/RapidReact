package frc.robot.controller;

import edu.wpi.first.wpilibj.DriverStation;

import java.util.HashMap;
import java.util.Map;

public class ControllerConfigIdentifier {

    static Map<String[], ControllerConfig> configMap = new HashMap<>();

    static {
        configMap.put(new String[]{"T.16000M", "T.16000M", "Logitech Dual Action"}, new JoysticksConfig());
        configMap.put(new String[]{"Logitech Dual Action", "Logitech Dual Action"}, new GamepadsConfig());
    }

    ControllerConfig getInferredControllerConfig() {
        for(Map.Entry entry : configMap.entrySet() ){
            String[] devices = (String[])entry.getKey();

            boolean matches = true;

            for(int i = 0; i < devices.length; i++) {
                if(!DriverStation.getJoystickName(i).equals(devices[i])){
                    matches = false;
                    break;
                }
            }

            if(matches) {
                return (ControllerConfig) entry.getValue();
            }
        }

        return new GamepadsConfig();
    }

}
