package frc.robot.helper.shooter;

public class ShooterState {
    public double rpmVelocity;
    public double hoodAngle;

    public ShooterState(double v, double t) {
        this.rpmVelocity = v;
        this.hoodAngle = t;
    }
}
