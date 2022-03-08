package frc.robot.helper.shooter;

import static frc.robot.Constants.ShooterConstants.*;

public class ShooterPresetSelector {
    public static ShooterPreset findClosesPreset(double distanceFromTarget) {
        double prevError = -1;
        for (int i = 0; i < ALL_SHOOTER_PRESETS.size(); i++) {
            ShooterPreset preset = ALL_SHOOTER_PRESETS.get(i);
            double error = Math.abs(preset.distanceFromTarget - distanceFromTarget);
            if (prevError != -1 && prevError < error) return ALL_SHOOTER_PRESETS.get(i-1);
            prevError = error;
        }
        return ALL_SHOOTER_PRESETS.get(ALL_SHOOTER_PRESETS.size()-1);
    }
}
