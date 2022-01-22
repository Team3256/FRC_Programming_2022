package frc.robot.commands;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.commands.shooter.SetShooterFromCustomState;
import frc.robot.subsystems.FlywheelSubsystem;
import org.junit.Before;
import org.junit.Test;

import static org.mockito.ArgumentMatchers.any;
import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.verify;

public class SetShooterFromCustomStateTest {
    private CommandScheduler commandScheduler;

    @Before
    public void setUp() {
        commandScheduler = CommandScheduler.getInstance();
        commandScheduler.cancelAll();
    }

    @Test
    public void customVelocityAndHoodTest() throws InterruptedException {
        //Way to trick the Command Scheduler into Running our Commands with no Robot
        //Java does not like it if this is placed in setUp Method
        DriverStationSim.setEnabled(true);

        FlywheelSubsystem flywheelSubsystem = mock(FlywheelSubsystem.class);
        SetShooterFromCustomState customStateShooterCommand = new SetShooterFromCustomState(flywheelSubsystem);

        commandScheduler.schedule(customStateShooterCommand);
        commandScheduler.run();

        verify(flywheelSubsystem).setSpeed(anyDouble());
        verify(flywheelSubsystem).setHoodAngle(anyDouble());
    }
}
