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

    private static final Color blueBall = new Color(0.143, 0.427, 0.429);
    private static final Color redBall = new Color(0.361, 0.524, 0.113);
    private static final int targetDistanceLimit = 200;

    /**
     * initialize colors in robot init
     */
    public static void loadColorMatcher() {
        colorMatcher.addColorMatch(blueBall);
        colorMatcher.addColorMatch(redBall);
    }

    /**
     * @return detected color
     */
    public static String getColorMatch() {
        Color detectedColor = colorSensor.getColor();
        ColorMatchResult match = colorMatcher.matchClosestColor(detectedColor);
        if (getProximity() > targetDistanceLimit) {
            String colorString;
            if (match.color == blueBall) {
                colorString = "Blue";
            } else if (match.color == redBall) {
                colorString = "Red";
            } else {
                colorString = "Unknown";
            }
            return colorString;
        }
        return "Out of Proximity";
    }


    private static Color getColor() {
        return colorSensor.getColor();
    }

    private static double getRawIR() {
        return colorSensor.getIR();
    }

    private static int getProximity() {
        return colorSensor.getProximity();
    }
}
