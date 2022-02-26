package frc.robot.helper;

public class ShooterPreset {
    public String presetName;
    public ShooterState currentState;
    public double distanceToTarget;

    public ShooterPreset(double v, double t, double d, String name) {
        this.presetName = name;
        currentState = new ShooterState(v, t);
        this.distanceToTarget = d;
    }
}
