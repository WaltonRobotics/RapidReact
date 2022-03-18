package frc.robot;

import frc.robot.robots.PracticeRapidReact;
import frc.robot.robots.WaltRobot;
import frc.robot.subsystems.Shooter;
import frc.robot.util.interpolation.InterpolatingDouble;
import org.junit.Test;

public class InterpolationTest {

    @Test
    public void testInterpolation() {
        WaltRobot robot = new PracticeRapidReact();

        double hoodAngle = robot.getShooterConfig().getHoodAngleMap(Shooter.AimTarget.HIGH_GOAL)
                .getInterpolated(new InterpolatingDouble(6.322)).value;

        System.out.println(hoodAngle);
    }

}
