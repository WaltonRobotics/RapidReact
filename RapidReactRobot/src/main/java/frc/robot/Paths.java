package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;

public class Paths {
    private static final double maxVelocity = 5.0; //m/s
    private static final double maxAcceleration = 2.5; //m/s^2
    // Name of path, max vel, max acc

//    public static PathPlannerTrajectory grabLowBlue2 =
//            PathPlanner.loadPath("grabLowBlue2", Units.feetToMeters(11.0), Units.feetToMeters(5.0));
//
//    public static PathPlannerTrajectory sCurveForward =
//            PathPlanner.loadPath("sCurveForward", Units.feetToMeters(11.0), Units.feetToMeters(5.0));

    public static final class TestTrajectories {


        public static final PathPlannerTrajectory fiveFeetForward =
                PathPlanner.loadPath("fiveFeetForward", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));

        public static final PathPlannerTrajectory sCurveForward =
                PathPlanner.loadPath("sCurveForward", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));

        public static final PathPlannerTrajectory sCurveBackward =
                PathPlanner.loadPath("sCurveBackward", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);

        public static final PathPlannerTrajectory halfFootBackwards =
                PathPlanner.loadPath("halfFootBackwards", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration),true);

    }

    public static final class RoutineOne {

        // Move approximately five feet off of tarmac
        public static final PathPlannerTrajectory gammaBackwards =
                PathPlanner.loadPath("gammaBackwards", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);

    }

    public static final class RoutineTwo {

        public static final PathPlannerTrajectory betaBackwards =
                PathPlanner.loadPath("betaBackwards", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);

    }

    public static final class RoutineThree {

        public static final PathPlannerTrajectory alphaPickUpA =
                PathPlanner.loadPath("alphaPickUpA", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);

    }

    public static final class RoutineFourA {

        public static final PathPlannerTrajectory betaPickUpB =
                PathPlanner.loadPath("betaPickUpB", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
        public static final PathPlannerTrajectory ballBShoottoballG =
                PathPlanner.loadPath("ballBShoottoballG", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
        public static final PathPlannerTrajectory ballGtoShoot =
                PathPlanner.loadPath("ballGtoShoot", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), false);

    }

    public static final class RoutineFiveA {

    }

    public static final class RoutineFiveB {

        public static final PathPlannerTrajectory ballAToBallB =
                PathPlanner.loadPath("ballAToBallB", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
        public static final PathPlannerTrajectory ballBtoShoot =
                PathPlanner.loadPath("ballBtoShoot", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), false);

    }

    public static final class RoutineFiveC {

        public static final PathPlannerTrajectory ballBToBallG =
                PathPlanner.loadPath("ballBToBallG", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
        public static final PathPlannerTrajectory ballGtoShoottoballC =
                PathPlanner.loadPath("ballGtoShoottoballC", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration),true);

    }

    public static final class RoutineFiveD {

        public static final PathPlannerTrajectory ballBToBallC =
                PathPlanner.loadPath("ballBToBallC", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));

    }

    public static final class RoutineSixG {

        public static final PathPlannerTrajectory gammaPickUpC =
                PathPlanner.loadPath("gammaPickUpC", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
        public static final PathPlannerTrajectory ballCToBallG =
                PathPlanner.loadPath("ballCToBallG", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));

    }

    public static final class RoutineSixA {

        public static final PathPlannerTrajectory ballAToBallG =
                PathPlanner.loadPath("ballAToBallG", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);

    }

    public static final class RoutineSeven {

        public static final PathPlannerTrajectory ballCToBallB =
                PathPlanner.loadPath("ballCToBallB", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
        public static final PathPlannerTrajectory ballBToBallA =
                PathPlanner.loadPath("ballBToBallA", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));

    }

    public static final class RoutineFiveFull {
        public static final PathPlannerTrajectory routineFiveBFull =
                PathPlanner.loadPath("routineFiveBFull", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
    }

}
