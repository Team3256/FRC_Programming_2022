package robot.helper;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.helper.DPadButton;
import org.junit.Before;
import org.junit.Test;

import static org.junit.Assert.*;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;

public class DPadButtonTest {

    XboxController xboxController = mock(XboxController.class);

    @Test
    public void upButtonIsPressedWhenDPadUpIsPressed() {
        //Get POV Returns the Angle of the Button Pressed
        //Goes in 45 degree increments, 0 = Up, 180 = Down,
        //Returns -1 when no buttons are pressed
        when(xboxController.getPOV()).thenReturn(0);

        DPadButton button = new DPadButton(xboxController, DPadButton.Direction.UP);

        assertTrue(button.get());
    }

    @Test
    public void upButtonIsNotPressedWhenDPadUpIsNotPressed() {
        when(xboxController.getPOV()).thenReturn(-1);

        DPadButton button = new DPadButton(xboxController, DPadButton.Direction.UP);

        assertFalse(button.get());
    }

    @Test
    public void downButtonIsPressedWhenDPadDownIsPressed() {
        when(xboxController.getPOV()).thenReturn(180);

        DPadButton button = new DPadButton(xboxController, DPadButton.Direction.DOWN);

        assertTrue(button.get());
    }

    @Test
    public void downButtonIsNotPressedWhenDPadDownIsNotPressed() {
        when(xboxController.getPOV()).thenReturn(-1);

        DPadButton button = new DPadButton(xboxController, DPadButton.Direction.DOWN);

        assertFalse(button.get());
    }

    @Test
    public void downButtonIsNotPressedWhenDPadLeftIsPressed() {
        when(xboxController.getPOV()).thenReturn(270);

        DPadButton button = new DPadButton(xboxController, DPadButton.Direction.DOWN);

        assertFalse(button.get());
    }
}