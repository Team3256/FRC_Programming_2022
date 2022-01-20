package frc.robot.commands;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SetCustomVelocityShooterCommand;
import frc.robot.commands.SetPercentSpeedShooterCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import org.junit.Before;
import org.junit.Test;

import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

public class StopShooterTest {
    private CommandScheduler commandScheduler;

    @Before
    public void setUp() {
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.cancelAll();
    }

    @Test
    public void stopShooterTest() throws Exception {
        //super.setUp();
        DriverStationSim.setEnabled(true);

        FlywheelSubsystem flywheelSubsystem = mock(FlywheelSubsystem.class); //create fake "robot"
        StopShooter stopShooterTester = new StopShooter(flywheelSubsystem); //gets command to be tested

        commandScheduler.schedule(stopShooterTester); //schedules command
        commandScheduler.run();

        verify(flywheelSubsystem).stop();

    }
}