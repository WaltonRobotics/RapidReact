package frc.robot.vision;
import com.revrobotics.ColorMatch;
import com.revrobotics.ColorMatchResult;
import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color;

public class ColorSensor {
    private static final I2C.Port i2cPort = I2C.Port.kOnboard;
    public static final ColorSensorV3 colorSensor = new ColorSensorV3(i2cPort);
    private static final ColorMatch colorMatcher = new ColorMatch();

    private static final Color blueTarget = new Color(0.143, 0.427, 0.429);
    private static final Color greenTarget = new Color(0.197, 0.561, 0.240);
    private static final Color redTarget = new Color(0.361, 0.524, 0.113);
    private static final int targetDistanceLimit = 200;

    public static void loadColorMatcher(){
        colorMatcher.addColorMatch(blueTarget);
        colorMatcher.addColorMatch(greenTarget);
        colorMatcher.addColorMatch(redTarget);
    }

    public static String getColorMatch() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        if (getProximity() > targetDistanceLimit) {
            String colorString;
            if (match.color == blueTarget) {
                colorString = "Blue";
            } else if (match.color == redTarget) {
                colorString = "Red";
            } else if (match.color == greenTarget) {
                colorString = "Green";
            } else {
                colorString = "Unknown";
            }
            return colorString;
        }
        return "Unknown";
    }

    public static boolean[] allAligned(){
        boolean[] statuses = new boolean [3];
        //TODO: check if all 3 sensors match the color and return array
        //use for loop to check color match for each sensor
        //set arrays indices to if each sensor matches (left to right)
        return statuses;

    }

    private static Color getColor(){
        return colorSensor.getColor();
    }

    private static double getRawIR(){
        return colorSensor.getIR();
    }

    private static int getProximity() {
        return colorSensor.getProximity();
    }
}
