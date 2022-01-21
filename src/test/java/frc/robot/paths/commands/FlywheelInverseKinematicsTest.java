package frc.robot.paths.commands;

import frc.robot.subsystems.FlywheelSubsystem;
import org.junit.Test;
import java.text.DecimalFormat;
import static org.junit.Assert.assertArrayEquals;

public class FlywheelInverseKinematicsTest {
    @Test
    public void checksOutputOfInverseKinematics() {
        FlywheelSubsystem flywheelSubsystem = new FlywheelSubsystem();
        double[] velocityAndEntryAngle = flywheelSubsystem.ballInverseKinematicsTester(4.4, 50);
        String[] velocityEntryString = new String[2];

        DecimalFormat df0 = new DecimalFormat("##.###########");
        velocityEntryString[0] = df0.format(velocityAndEntryAngle[0]);

        DecimalFormat df1 = new DecimalFormat("##.##########");
        velocityEntryString[1] = df1.format(velocityAndEntryAngle[1]);

        String[] accurateCompareTo = {"8.93681763198", "64.2817680352"};

        assertArrayEquals(accurateCompareTo, velocityEntryString);
    }
}
