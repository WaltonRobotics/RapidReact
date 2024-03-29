package frc.robot;

import frc.robot.robots.PracticeRapidReact;
import frc.robot.robots.WaltRobot;
import frc.robot.subsystems.Shooter;
import frc.robot.util.UtilMethods;
import frc.robot.util.interpolation.InterpolatingDouble;
import org.junit.Assert;
import org.junit.Test;

public class InterpolationTest {

    @Test
    public void testInterpolation() {
        WaltRobot robot = new PracticeRapidReact();

        double hoodAngle = robot.getShooterConfig().getHoodAngleMap(Shooter.AimTarget.HIGH_GOAL)
                .getInterpolated(new InterpolatingDouble(7.56)).value;

        double rpm = robot.getShooterConfig().getFlywheelVelocityMap(Shooter.AimTarget.HIGH_GOAL)
                .getInterpolated(new InterpolatingDouble(7.56)).value;

//        Assert.assertEquals(hoodAngle, -0.189, 0.1);

        System.out.println(hoodAngle);
        System.out.println(rpm);

        double test = UtilMethods.restrictAngle(181, -180, 180);
        System.out.println(test);
    }

}
