package frc.robot.helper;

import junit.framework.TestCase;
import org.junit.Assert;
import org.junit.Test;

import static frc.robot.helper.SmoothVelocity.smoothVelocity;
import static frc.robot.helper.SmoothVelocity.RobotKinematicState;

public class SmoothVelocityTest {

    @Test
    public void smoothVelocityMaintainsTargetOnIncrease() {
        RobotKinematicState robotKinematicState = new RobotKinematicState(0,0);

        RobotKinematicState  newState = smoothVelocity(robotKinematicState, 1, 0.01, 1);

        Assert.assertEquals(1, newState.velocity, 0.01);

    }
    @Test
    public void smoothVelocityMaintainsTargetOnDecrease() {
        RobotKinematicState robotKinematicState = new RobotKinematicState(0,0);

        RobotKinematicState  newState = smoothVelocity(robotKinematicState, -1, 0.01, 1);

        Assert.assertEquals(-1, newState.velocity, 0.01);
    }
    @Test
    public void smoothVelocitySlowlyDecreasesWhenDecelerating() {
        RobotKinematicState robotKinematicState = new RobotKinematicState(0,0);

        robotKinematicState = smoothVelocity(robotKinematicState, 1, 0.01, 1);
        robotKinematicState = smoothVelocity(robotKinematicState, 0, 0.01, 1);

        Assert.assertNotEquals(0, robotKinematicState.velocity, 0.01);
    }
    @Test
    public void smoothVelocityDecreasesWhenDeceleratingPast0() {
        RobotKinematicState robotKinematicState = new RobotKinematicState(0,0);


        robotKinematicState = smoothVelocity(robotKinematicState, 1, 0.01, 1);
        for (int i =0; i < 1000; i++) {
            robotKinematicState = smoothVelocity(robotKinematicState, -1, 0.1, 100);
        }

        Assert.assertEquals(-1, robotKinematicState.velocity, 0.01);
    }
}