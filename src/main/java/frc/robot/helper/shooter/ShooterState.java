package frc.robot.helper.shooter;

public class ShooterState {
    public double rpmVelocity; // in rpm
    public double hoodAngle; // in motorSensorUnits

    public ShooterState(double v, double t) {
        this.rpmVelocity = v;
        this.hoodAngle = t;
    }
}
