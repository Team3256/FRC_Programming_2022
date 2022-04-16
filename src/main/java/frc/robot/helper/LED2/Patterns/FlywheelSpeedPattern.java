package frc.robot.helper.LED2.Patterns;

import frc.robot.helper.LED2.Color;
import frc.robot.subsystems.ShooterSubsystem;

public class FlywheelSpeedPattern extends LEDPattern {
    public FlywheelSpeedPattern(Color[] totalPattern) {
        super(50, 99, totalPattern);
    }

    @Override
    public void update(){
        super.update();
        int flywheelSpeed = 100;
        int maxFlywheelSpeed = 500;
        setRange(0, 50*flywheelSpeed/maxFlywheelSpeed, new Color(0,255,0));
    }
}
