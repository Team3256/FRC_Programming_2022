package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FlywheelSubsystem.ShooterLocationPreset;

public class SetShooterPreset extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    private ShooterLocationPreset preset;

    public SetShooterPreset(FlywheelSubsystem flywheelSubsystem, ShooterLocationPreset preset) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.preset = preset;
    }

    @Override
    public void initialize() {
        flywheelSubsystem.setShooterLocationPreset(preset);
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}


