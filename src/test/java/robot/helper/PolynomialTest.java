package robot.helper;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.helper.FileUtil;
import frc.robot.helper.Polynomial;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;

public class PolynomialTest {
    private CommandScheduler scheduler;

    @Before
    public void setUp(){
        scheduler = CommandScheduler.getInstance();
        scheduler.cancelAll();
    }

    @Test
    public void checkPolynomialEquals(){
        Polynomial testPolynomial1 = new Polynomial(new double[]{3,2,5,6});
        Polynomial testPolynomial2 = new Polynomial(new double[]{3,2,5,6});
        Polynomial testPolynomial3 = new Polynomial(new double[]{2,5,4});
        assertTrue(testPolynomial1.equals(testPolynomial2));
        assertFalse(testPolynomial1.equals(testPolynomial3));
    }

    @Test
    public void checkPolynomialOutput(){
        Polynomial testPolynomial = new Polynomial(new double[]{3,2,5,6});
        //8*6 + 4*5 + 2*2 +3 = 75
        assertTrue(testPolynomial.getOutput(2)==75);
    }

    @Test
    public void checkPolynomialIO(){
        Polynomial testWritePolynomial = new Polynomial(new double[]{3,2,5,6});
        FileUtil.writeObjectToFile("Polynomial.txt", testWritePolynomial);
        Polynomial testReadPolynomial = (Polynomial) FileUtil.readObjectFromFile("Polynomial.txt");
        assertTrue(testReadPolynomial.getOutput(2)==75);
    }

    @Test
    public void checkPolynomialDefault(){
        Polynomial testReadPolynomial;
    }
}
