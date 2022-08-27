package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ShooterSubsystem.ShooterLocationPreset;

public class SetShooterPreset extends CommandBase {
    private ShooterSubsystem shooterSubsystem;
    private ShooterLocationPreset preset;

    public SetShooterPreset(ShooterSubsystem flywheelSubsystem, ShooterLocationPreset preset) {
        this.shooterSubsystem = shooterSubsystem;
        this.preset = preset;
    }

    @Override
    public void initialize() {
        // shooterSubsystem.setShooterLocationPreset(preset);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
