package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;

public class Paths {
    // Name of path, max vel, max acc

//    public static PathPlannerTrajectory grabLowBlue2 =
//            PathPlanner.loadPath("grabLowBlue2", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
//
//    public static PathPlannerTrajectory sCurveForward =
//            PathPlanner.loadPath("sCurveForward", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

    public static final class TestTrajectories {

        public static final PathPlannerTrajectory fiveFeetForward =
                PathPlanner.loadPath("fiveFeetForward", Units.feetToMeters(5), Units.feetToMeters(4));

        public static final PathPlannerTrajectory sCurveForward =
                PathPlanner.loadPath("sCurveForward", Units.feetToMeters(5.5), Units.feetToMeters(2.5));

        public static final PathPlannerTrajectory sCurveBackward =
                PathPlanner.loadPath("sCurveBackward", Units.feetToMeters(5.5), Units.feetToMeters(2.5), true);

    }

    public static final class RoutineOne {

        // Move approximately five feet off of tarmac
        public static final PathPlannerTrajectory backwards =
                PathPlanner.loadPath("backwards", Units.feetToMeters(8), Units.feetToMeters(4), true);

    }

    public static final class RoutineTwo {

        public static final PathPlannerTrajectory betaBackward =
                PathPlanner.loadPath("betaBackward", Units.feetToMeters(8), Units.feetToMeters(4), true);

    }

    public static final class RoutineThree {

        public static final PathPlannerTrajectory alphaPickUpA =
                PathPlanner.loadPath("alphaPickUpA", Units.feetToMeters(8), Units.feetToMeters(4), true);

    }

    public static final class RoutineFourA {

        public static final PathPlannerTrajectory betaPickUpB =
                PathPlanner.loadPath("betaPickUpB", Units.feetToMeters(8), Units.feetToMeters(4), true);
        //TODO: Finish this routine

    }

    public static final class RoutineFiveA {

    }

    public static final class RoutineFiveB {

        public static final PathPlannerTrajectory ballAToBallB =
                PathPlanner.loadPath("ballAToBallB", Units.feetToMeters(8), Units.feetToMeters(4), true);

    }

    public static final class RoutineFiveC {

        public static final PathPlannerTrajectory ballBToBallG =
                PathPlanner.loadPath("ballBToBallG", Units.feetToMeters(8), Units.feetToMeters(4), true);

    }

    public static final class RoutineFiveD {

        public static final PathPlannerTrajectory ballBToBallC =
                PathPlanner.loadPath("ballBToBallC", Units.feetToMeters(8), Units.feetToMeters(4));

    }

    public static final class RoutineSixG {

        public static final PathPlannerTrajectory gammaPickUpC =
                PathPlanner.loadPath("gammaPickUpC", Units.feetToMeters(8), Units.feetToMeters(4), true);
        public static final PathPlannerTrajectory ballCToBallG =
                PathPlanner.loadPath("ballCToBallG", Units.feetToMeters(8), Units.feetToMeters(4));

    }

    public static final class RoutineSixA {

        public static final PathPlannerTrajectory ballAToBallG =
                PathPlanner.loadPath("ballAToBallG", Units.feetToMeters(8), Units.feetToMeters(4), true);

    }

    public static final class RoutineSeven {

        public static final PathPlannerTrajectory ballCToBallB =
                PathPlanner.loadPath("ballCToBallB", Units.feetToMeters(8), Units.feetToMeters(4));
        public static final PathPlannerTrajectory ballBToBallA =
                PathPlanner.loadPath("ballBToBallA", Units.feetToMeters(8), Units.feetToMeters(4));

    }

}
