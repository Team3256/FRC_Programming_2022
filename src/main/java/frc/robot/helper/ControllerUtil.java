package frc.robot.helper;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ControllerUtil {
    public static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            return value;
//            if (value > 0.0) {
//                return (value - deadband) / (1.0 - deadband);
//            } else {
//                return (value + deadband) / (1.0 - deadband);
//            }
        } else {
            return 0.0;
        }
    }

    public static double modifyAxis(double value) {
        double deadband = 0.05;
        value = deadband(value, deadband);

        if (value == 0) {
            return 0;
        }

        SmartDashboard.setDefaultNumber("Joystick Input Exponential Power", 3);
//
        double exp = SmartDashboard.getNumber("Joystick Input Exponential Power", 3);
        value = Math.copySign(Math.pow((((1 + deadband)*value) - deadband), exp), value);

        return value;
    }
}
