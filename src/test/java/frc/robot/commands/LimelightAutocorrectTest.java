package frc.robot.commands;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.SwerveDrive;
import org.junit.Before;
import org.junit.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

import static org.mockito.Mockito.*;

public class LimelightAutocorrectTest {
    private CommandScheduler scheduler;

    @Before
    public void setUp(){
        scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
    }

    @Test
    public void checkPolynomial() {
        DriverStationSim.setEnabled(true);
        SwerveDrive swerveDrive = mock(SwerveDrive.class);
        try (MockedStatic<Limelight> limelight = Mockito.mockStatic(Limelight.class)) {
            limelight.when(Limelight::getDistanceToTarget).thenReturn(Math.random()*500);
        }

        LimelightAutocorrectCommand command = new LimelightAutocorrectCommand(swerveDrive, 500);
        scheduler.schedule();

        scheduler.run();

    }
}
