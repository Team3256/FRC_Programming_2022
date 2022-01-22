package frc.robot.commands;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shooter.AutoAimShooter;
import frc.robot.subsystems.FlywheelSubsystem;
import org.junit.Before;
import org.junit.Test;

import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

public class AutoAimShooterTest {
    private CommandScheduler commandScheduler;

    @Before
    public void setUp() {
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.cancelAll();
    }

    @Test
    public void setsCustomVelocityShooter() throws InterruptedException {
        //Way to trick the Command Scheduler into Running our Commands with no Robot
        //Java does not like it if this is placed in setUp Method
        DriverStationSim.setEnabled(true);

        FlywheelSubsystem flywheelSubsystem = mock(FlywheelSubsystem.class);
        AutoAimShooter customVelocityCommand = new AutoAimShooter(flywheelSubsystem);

        commandScheduler.schedule(customVelocityCommand);
        commandScheduler.run();

        verify(flywheelSubsystem).autoAim(anyDouble(), anyDouble());
    }
}
