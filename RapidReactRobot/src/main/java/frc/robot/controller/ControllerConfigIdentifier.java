package frc.robot.controller;

public class ControllerConfigIdentifier {

    ControllerConfig getInferredControllerConfig() {
        //return new GamepadsConfig();
        return new XboxConfig();
    }

}
