package frc.robot.paths.commands;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.SetCustomVelocityShooterCommand;
import frc.robot.commands.SetPercentSpeedShooterCommand;
import frc.robot.subsystems.FlywheelSubsystem;
import org.junit.Before;
import org.junit.Test;

import java.util.function.DoubleSupplier;

import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

public class SetShooterTriggerVelocityTest {
    private CommandScheduler commandScheduler;

    @Before
    public void setUp() {
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.cancelAll();
    }

    @Test
    public void setTriggerVelocityTest() throws InterruptedException {
        //Way to trick the Command Scheduler into Running our Commands with no Robot
        //Java does not like it if this is placed in setUp Method
        DriverStationSim.setEnabled(true);

        FlywheelSubsystem flywheelSubsystem = mock(FlywheelSubsystem.class);
        SetPercentSpeedShooterCommand percentSpeedCommand = new SetPercentSpeedShooterCommand(flywheelSubsystem, ()->0.4);

        commandScheduler.schedule(percentSpeedCommand);
        commandScheduler.run();

        verify(flywheelSubsystem).setPercentSpeed(0.4 * 100);
    }
}
