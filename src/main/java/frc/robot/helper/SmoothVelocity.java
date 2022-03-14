package frc.robot.helper;

public class SmoothVelocity {
    public static double smoothVelocity(double currentVelocity, double targetVelocity, double acceleration, double time) {
        double currentAcceleration = (targetVelocity - currentVelocity)/time;
        return currentVelocity  + (Math.min(Math.abs(currentAcceleration), acceleration)) * time;
    }
}
