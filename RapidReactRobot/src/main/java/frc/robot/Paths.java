package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;

public class Paths {

//    public static PathPlannerTrajectory grabLowBlue2 =
//            PathPlanner.loadPath("grabLowBlue2", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
//
//    public static PathPlannerTrajectory sCurveForward =
//            PathPlanner.loadPath("sCurveForward", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

    public static class RoutineOne {
        // Move approximately 5m diagonal off of tarmac
        // start at gamma with shooter facing the hub
        public static PathPlannerTrajectory gammaMoveOffTarmac =
                PathPlanner.loadPath("gammaMoveOffTarmac", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

    }

    public static class RoutineTwo {

    }

    public static class RoutineThree{
        public static PathPlannerTrajectory alphaPickUpA =
                PathPlanner.loadPath("alphaPickUpA", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
    }
}
