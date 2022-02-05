package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Superstructure;

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
                PathPlanner.loadPath("alphaPickUpA", Units.feetToMeters(11.0), Units.feetToMeters(5.0), true);
    }

    public static class RoutineFiveB{
        public static PathPlannerTrajectory alphaPickUpA =
                PathPlanner.loadPath("alphaPickUpA", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
        public static PathPlannerTrajectory ballAtoBallB=
                PathPlanner.loadPath("ballAtoballB", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
        public static PathPlannerTrajectory ballBtoBallG=
                PathPlanner.loadPath("ballBtoballG", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
        //MOVE IN
    }

    public static class RoutineFourA{
        public static PathPlannerTrajectory betaPickUpB=
                PathPlanner.loadPath("betaPickUpB", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
        //TODO: Finish this routine
    }

    public static class RoutineSix{
        public static PathPlannerTrajectory gammaPickUpC =
                PathPlanner.loadPath("gammaPickUpC", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
        public static PathPlannerTrajectory ballCtoBallG =
                PathPlanner.loadPath("ballCtoballG", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
        //MOVE IN
    }
}
