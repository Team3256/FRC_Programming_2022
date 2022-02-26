package frc.robot.helper.shooter;

public class ShooterPreset {
    public String presetName;
    public ShooterState currentState;

    /**
     * @param velocity velocity in rpm (velocity)
     * @param hoodAngleTheta hoodAngle in motorSensorUnits (theta)
     * @param name preset name to be given to the operator on selection
     */
    public ShooterPreset(double velocity, double hoodAngleTheta, String name) {
        this.presetName = name;
        currentState = new ShooterState(velocity, hoodAngleTheta);
    }
}
