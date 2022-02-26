package frc.robot.helper.shooter;

public class ShooterPreset {
    public String presetName;
    public ShooterState currentState;

    /**
     * @param v velocity in rpm (velocity)
     * @param t hoodAngle in motorSensorUnits (theta)
     * @param name preset name to be given to the operator on selection
     */
    public ShooterPreset(double v, double t, String name) {
        this.presetName = name;
        currentState = new ShooterState(v, t);
    }
}
