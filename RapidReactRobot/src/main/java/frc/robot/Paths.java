package frc.robot;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.util.Units;

public class Paths {

    private static final double maxVelocity = 9.5; //ft/s
    private static final double maxAcceleration = 4.5; //ft/s^2
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
                PathPlanner.loadPath("halfFootBackwards", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);

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

//    public static final class RoutineThree {
//
//        public static final PathPlannerTrajectory alphaPickUpA =
//                PathPlanner.loadPath("alphaPickUpA", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
//
//    }
//
//    public static final class RoutineFourA {
//
//        public static final PathPlannerTrajectory betaPickUpB =
//                PathPlanner.loadPath("betaPickUpB", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
//        public static final PathPlannerTrajectory ballBShootToBallG =
//                PathPlanner.loadPath("ballBShootToBallG", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
//        public static final PathPlannerTrajectory ballGToShoot =
//                PathPlanner.loadPath("ballGToShoot", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), false);
//        public static final PathPlannerTrajectory ballGToBallB =
//                PathPlanner.loadPath("ballGToBallB", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
//
//    }
//
//    public static final class RoutineFiveA {
//
//    }
//
//    public static final class RoutineFiveB {
//
//        public static final PathPlannerTrajectory ballAToBallB =
//                PathPlanner.loadPath("ballAToBallB", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
//        public static final PathPlannerTrajectory ballBToShoot =
//                PathPlanner.loadPath("ballBToShoot", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), false);
//        public static final PathPlannerTrajectory alphaPickUpA2 =
//                PathPlanner.loadPath("alphaPickUpA2", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
//        public static final PathPlannerTrajectory ballAToBallB2 =
//                PathPlanner.loadPath("ballAToBallB2", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
//
//
//    }
//
//    public static final class RoutineFiveC {
//
//        public static final PathPlannerTrajectory ballBToBallG =
//                PathPlanner.loadPath("ballBToBallG", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
//        public static final PathPlannerTrajectory ballGToShootToBallC =
//                PathPlanner.loadPath("ballGToShootToBallC", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
//
//    }
//
//    public static final class RoutineFiveD {
//
//        public static final PathPlannerTrajectory ballBToBallC =
//                PathPlanner.loadPath("ballBToBallC", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
//
//    }
//
//    public static final class RoutineFiveE {    //money shot
//        public static final PathPlannerTrajectory ballBToMoneyShot =
//                PathPlanner.loadPath("ballBToMoneyShot", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
//    }

    public static final class RoutineSixG {

        public static final PathPlannerTrajectory gammaPickUpC =
                PathPlanner.loadPath("gammaPickUpC", Units.feetToMeters(4.0), Units.feetToMeters(maxAcceleration), true);
//        public static final PathPlannerTrajectory ballCToBallG =
//                PathPlanner.loadPath("ballCToBallG", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
//        public static final PathPlannerTrajectory ballCToShoot =
//                PathPlanner.loadPath("ballCToBallG", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));

    }

//    public static final class RoutineSixA {
//
//        public static final PathPlannerTrajectory ballAToBallG =
//                PathPlanner.loadPath("ballAToBallG", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration), true);
//
//    }
//
//    public static final class RoutineSeven {
//
//        public static final PathPlannerTrajectory ballCToBallB =
//                PathPlanner.loadPath("ballCToBallB", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
//        public static final PathPlannerTrajectory ballBToBallA =
//                PathPlanner.loadPath("ballBToBallA", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
//        public static final PathPlannerTrajectory ballAToShoot =
//                PathPlanner.loadPath("ballAToShoot", Units.feetToMeters(maxVelocity), Units.feetToMeters(maxAcceleration));
//
//    }

    public static final class RoutineFiveFull {

        public static final PathPlannerTrajectory routineFiveBFull =
                PathPlanner.loadPath("routineFiveBFull", Units.feetToMeters(11.0), Units.feetToMeters(4.5), true);

        public static final PathPlannerTrajectory pickupGShoot =
                PathPlanner.loadPath("pickupGShoot", Units.feetToMeters(11), 1.75);

        public static final PathPlannerTrajectory routineFiveBFullFast =
                PathPlanner.loadPath("routineFiveBFull", Units.feetToMeters(11.0), 1.75, true);
    }

    public static final class RoutineEight {

//        public static final PathPlannerTrajectory pickupAAndB =
//                PathPlanner.loadPath("pickupAAndB", Units.feetToMeters(8), Units.feetToMeters(4), true);

        public static final PathPlannerTrajectory pickupG =
                PathPlanner.loadPath("pickupG", Units.feetToMeters(11.0), Units.feetToMeters(4.5), true);

        public static final PathPlannerTrajectory pickupGFast =
                PathPlanner.loadPath("pickupG", Units.feetToMeters(11.0), 1.75, true);

    }

}
