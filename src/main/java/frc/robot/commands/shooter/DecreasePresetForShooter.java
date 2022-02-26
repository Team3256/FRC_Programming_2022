package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class DecreasePresetForShooter extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public DecreasePresetForShooter(FlywheelSubsystem m_flywheelSubsystem) {
        flywheelSubsystem = m_flywheelSubsystem;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.decreasePreset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
