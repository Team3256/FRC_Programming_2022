package frc.robot.helper.shooter;

import static frc.robot.Constants.ShooterConstants.ALL_SHOOTER_PRESETS;

public class PresetManager {
    private int currentPresetNumber = 0;
    private ShooterLocationPreset shooterLocationPreset = ShooterLocationPreset.FENDER;

    public void increasePreset() {
        currentPresetNumber++;
        if (currentPresetNumber >= ALL_SHOOTER_PRESETS.size()) currentPresetNumber = 0;
    }

    public void decreasePreset(){
        currentPresetNumber--;
        if (currentPresetNumber < 0) currentPresetNumber = ALL_SHOOTER_PRESETS.size()-1;
    }

    public ShooterState getFlywheelShooterStateFromPreset(){
        return ALL_SHOOTER_PRESETS.get(shooterLocationPreset).shooterState;
    }
}
