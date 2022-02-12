package robot.helper;

import edu.wpi.first.wpilibj.simulation.DriverStationSim;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helper.Limelight;
import frc.robot.helper.Polynomial;
import frc.robot.subsystems.SwerveDrive;
import org.junit.Before;
import org.junit.Test;
import org.mockito.MockedStatic;
import org.mockito.Mockito;

import java.util.logging.Logger;

public class LimelightTest {
    private CommandScheduler scheduler;

    @Before
    public void setUp(){
        scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
    }

    @Test
    public void checkWritePolynomial(){
        Polynomial testPolynomial = new Polynomial(new double[]{1,0,0,1});
        try {
            Limelight.writePolynomial(testPolynomial);
        } catch (Exception e) {
            System.out.println(e);
        }
    }
}
