package frc.robot.helper.shooter;

public class ShooterPreset {
    public String presetName;
    public ShooterState shooterState;
    public double distanceFromTarget;

    /**
     * @param velocity velocity in rpm (velocity)
     * @param hoodAngleTheta hoodAngle in motorSensorUnits (theta)
     * @param name preset name to be given to the operator on selection
     */
    public ShooterPreset(double velocity, double hoodAngleTheta, double distanceFromTarget, String name) {
        this.presetName = name;
        this.distanceFromTarget = distanceFromTarget;
        shooterState = new ShooterState(velocity, hoodAngleTheta);
    }
}
