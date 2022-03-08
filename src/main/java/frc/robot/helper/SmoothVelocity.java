package frc.robot.helper;

public class SmoothVelocity {

    public void SmoothVelocity() {
    }

    public double smooth(double currentVelocity, double targetVelocity, double acceleration, double time) {
        double delta = targetVelocity - currentVelocity;
        if (Math.abs(delta) < acceleration) {
            return targetVelocity;
        } else {
            return currentVelocity + Math.signum(delta) * acceleration * time;
        }
    }
}
