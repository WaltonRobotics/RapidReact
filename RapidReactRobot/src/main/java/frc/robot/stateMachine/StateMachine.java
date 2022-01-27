package frc.robot.stateMachine;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.logging.Level;

import static frc.robot.RobotContainer.robotLogger;
import static frc.robot.util.UtilMethods.joinStrings;

public class StateMachine {

    private final String name;
    private IState currentState;

    public StateMachine(String name, IState initialState) {
        this.name = name;

        if (initialState != null) {
            currentState = initialState;
        } else {
            throw new IllegalArgumentException("Initial state must not be null!");
        }

        currentState.initialize();
    }

    public void run() {
        if (currentState.getName() == null) {
            robotLogger.log(Level.WARNING, currentState.getName() + " state has a null name!");
        } else {
            SmartDashboard.putString(joinStrings(" ", name, "Current State"), currentState.getName());
        }

        IState nextState = currentState.execute();

        if (nextState != null) {
            if (!nextState.equals(currentState)) {
                currentState.finish();
                currentState = nextState;
                currentState.initialize();
            }
        } else {
            robotLogger.log(Level.WARNING,
                    "State machine \"" + name + "\" has effectively terminated due to a null state");
        }
    }

    public IState getCurrentState() {
        return currentState;
    }

    public String getName() {
        return name;
    }

}

