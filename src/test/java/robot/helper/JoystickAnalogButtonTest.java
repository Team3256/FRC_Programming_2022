package robot.helper;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.simulation.GenericHIDSim;
import edu.wpi.first.wpilibj.simulation.XboxControllerSim;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.helper.JoystickAnalogButton;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.*;

public class JoystickAnalogButtonTest {
    XboxController xboxController = mock(XboxController.class);


    //Default Threshold is 0.5
    @Test
    public void buttonDoesNotTriggerWhenBelowDefaultThreshold() {

        Button digitalButton = new JoystickAnalogButton(xboxController, 0);

        doAnswer(invocation->0.2).when(xboxController).getRawAxis(0);
        assertFalse(digitalButton.get());
    }

    @Test
    public void buttonDoesTriggerWhenAboveDefaultThreshold() {
        Button digitalButton = new JoystickAnalogButton(xboxController, 0);

        doAnswer(invocation->0.95).when(xboxController).getRawAxis(0);
        assertTrue(digitalButton.get());
    }

    @Test
    public void buttonDoesTriggerWhenAboveSetThresholdThroughMethod() {
        JoystickAnalogButton digitalButton = new JoystickAnalogButton(xboxController, 0);

        digitalButton.setThreshold(0.2);

        doAnswer(invocation->0.3).when(xboxController).getRawAxis(0);
        assertTrue(digitalButton.get());

    }

    @Test
    public void buttonDoesTriggerWhenAboveSetThresholdThroughConstructor() {
        JoystickAnalogButton digitalButton = new JoystickAnalogButton(xboxController, 0, 0.2);

        doAnswer(invocation->0.3).when(xboxController).getRawAxis(0);
        assertTrue(digitalButton.get());
    }

    @Test
    public void buttonReturnsCorrectThreshold() {
        JoystickAnalogButton digitalButton = new JoystickAnalogButton(xboxController, 0, 0.2);

        assertEquals(digitalButton.getThreshold(), 0.2, 0.01);
    }

    @Test
    public void buttonDoesTriggerWhenBelowSetNegativeThreshold() {
        JoystickAnalogButton digitalButton = new JoystickAnalogButton(xboxController, 0, -0.2);

        doAnswer(invocation->-0.4).when(xboxController).getRawAxis(0);
        assertTrue(digitalButton.get());
    }
}
