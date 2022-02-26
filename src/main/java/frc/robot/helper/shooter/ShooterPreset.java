package frc.robot.helper.shooter;

public class ShooterPreset {
    public String presetName;
    public ShooterState currentState;

    public ShooterPreset(double v, double t, String name) {
        this.presetName = name;
        currentState = new ShooterState(v, t);
    }
}
