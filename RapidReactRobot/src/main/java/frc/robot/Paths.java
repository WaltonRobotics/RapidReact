package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;

public class Paths {
    
    public static PathPlannerTrajectory grabLowBlue1 =
            PathPlanner.loadPath("grabLowBlue1", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

    public static PathPlannerTrajectory grabLowBlue2 =
            PathPlanner.loadPath("grabLowBlue2", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

    public static PathPlannerTrajectory sCurveForward =
            PathPlanner.loadPath("sCurveForward", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

}
