package frc.robot.vision;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {
    private static final I2C.Port i2cPort = I2C.Port.kOnboard;
    public static final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);


   // private static final ColorMatch colorMatcher = new ColorMatch(); //used to register and detect known colors

//    private static final Color blueTarget = ColarMatch.addColorMatch(0.143, 0.427, 0.429);
//    private static final Color greenTarget = colarMatcher.matchColor(0.197, 0.561, 0.240);
//    private static final Color redTarget = ColorMatch.makeColor(0.561, 0.232, 0.114);
//    private static final Color yellowTarget = colorMatcher.makeColor(0.361, 0.524, 0.113);
    private static final int targetDistanceLimit = 200;

//    public static String getColorMatch() {
//        Color detectedColor = colorSensor.getColor();
//        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
//        if (getProximity() > targetDistanceLimit) {
//            String colorString;
//            if (match.color == blueTarget) {
//                colorString = "Blue";
//            } else if (match.color == redTarget) {
//                colorString = "Red";
//            } else if (match.color == greenTarget) {
//                colorString = "Green";
//            } else if (match.color == yellowTarget) {
//                colorString = "Yellow";
//            } else {
//                colorString = "Unknown";
//            }
//            return colorString;
//        }
//        return "Unknown";
//    }

    private static Color getColor(){
        return colorSensor.getColor();
    }

    private static double getRawIR(){
        return colorSensor.getIR();
    }

//    private static ColorMatch loadColorMatcher() {
//        ColorMatch colorMatcher = new ColorMatch();
//        colorMatcher.addColorMatch(blueTarget);
//        colorMatcher.addColorMatch(greenTarget);
//        colorMatcher.addColorMatch(redTarget);
//        colorMatcher.addColorMatch(yellowTarget);
//        return colorMatcher;
//    }

    private static int getProximity() {
        return colorSensor.getProximity();
    }
}
