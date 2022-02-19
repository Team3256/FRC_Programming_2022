package frc.robot.subsystems;

import com.revrobotics.ColorSensorV3;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class ColorsensorTestSubsystem extends SubsystemBase {
    private final I2C.Port i2cPort;
    private final ColorSensorV3 colorSensor;
    private final boolean debug = true;

    public ColorsensorTestSubsystem(){
        i2cPort = I2C.Port.kOnboard;
        colorSensor = new ColorSensorV3(i2cPort);
    }
    @Override
    public void periodic(){
        if (debug) postColor();
        else postTapeDetected();
    }
    public void postColor(){
        Color detectedColor = colorSensor.getColor();
        SmartDashboard.putNumber("Red", detectedColor.red);
        SmartDashboard.putNumber("Green", detectedColor.green);
        SmartDashboard.putNumber("Blue", detectedColor.blue);
    }
    public void postTapeDetected(){
        Color detectedColor = colorSensor.getColor();
    }
}
