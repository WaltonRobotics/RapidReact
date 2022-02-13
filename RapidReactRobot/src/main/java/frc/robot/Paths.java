package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;
import frc.robot.subsystems.Superstructure;

public class Paths {
    //name of path, max vel, max acc

//    public static PathPlannerTrajectory grabLowBlue2 =
//            PathPlanner.loadPath("grabLowBlue2", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
//
//    public static PathPlannerTrajectory sCurveForward =
//            PathPlanner.loadPath("sCurveForward", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

    public static PathPlannerTrajectory fiveFeetForward =
            PathPlanner.loadPath("fiveFeetForward", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

    public static PathPlannerTrajectory sCurveForward =
            PathPlanner.loadPath("sCurveForward", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

    public static PathPlannerTrajectory sCurveBackward =
            PathPlanner.loadPath("sCurveBackward", Units.feetToMeters(11.0), Units.feetToMeters(5.0), true);


    public static class RoutineOne {
        // Move approximately 5m diagonal off of tarmac
        // start at gamma with shooter facing the hub
        public static PathPlannerTrajectory gammaMoveOffTarmac =
                PathPlanner.loadPath("gammaMoveOffTarmac", Units.feetToMeters(11.0), Units.feetToMeters(5.0), true);

    }

    public static class RoutineTwo {
        public static PathPlannerTrajectory betaBackward =
                PathPlanner.loadPath("betaBackward", Units.feetToMeters(11.0), Units.feetToMeters(5.0), true);
    }

    public static class RoutineThree{
        public static PathPlannerTrajectory alphaPickUpA =
                PathPlanner.loadPath("alphaPickUpA", Units.feetToMeters(11.0), Units.feetToMeters(5.0), true);
    }

    public static class RoutineFourA{
        public static PathPlannerTrajectory betaPickUpB=
                PathPlanner.loadPath("betaPickUpB", Units.feetToMeters(11.0), Units.feetToMeters(5.0), true);
        //TODO: Finish this routine
    }

    public static class RoutineFiveA{
        PathPlannerTrajectory alphaPickUpA;
    }

    public static class RoutineFiveB{
        PathPlannerTrajectory alphaPickUpA;
        public static PathPlannerTrajectory ballAtoBallB=
                PathPlanner.loadPath("ballAtoballB", Units.feetToMeters(11.0), Units.feetToMeters(5.0), true);
        //MOVE IN
    }

    public static class RoutineFiveC{
        PathPlannerTrajectory alphaPickUpA;
        PathPlannerTrajectory ballAtoBallB;
        public static PathPlannerTrajectory ballBtoBallG=
                PathPlanner.loadPath("ballBtoballG", Units.feetToMeters(11.0), Units.feetToMeters(5.0),true);
    }

    public static class RoutineFiveD{
        PathPlannerTrajectory alphaPickUpA;
        PathPlannerTrajectory ballAtoBallB;
        //TODO: make the path for B to C
        public static PathPlannerTrajectory ballBtoballC =
                PathPlanner.loadPath("ballBtoballC", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
    }


    public static class RoutineSix{
        public static PathPlannerTrajectory gammaPickUpC =
                PathPlanner.loadPath("gammaPickUpC", Units.feetToMeters(11.0), Units.feetToMeters(5.0),true);
        public static PathPlannerTrajectory ballCtoBallG =
                PathPlanner.loadPath("ballCtoballG", Units.feetToMeters(11.0), Units.feetToMeters(5.0),true);
        //MOVE IN
    }
}
