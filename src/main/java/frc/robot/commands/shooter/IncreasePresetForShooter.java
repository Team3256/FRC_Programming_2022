package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

import java.awt.*;
import java.util.logging.Logger;

public class IncreasePresetForShooter extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public IncreasePresetForShooter(FlywheelSubsystem m_flywheelSubsystem) {
        flywheelSubsystem = m_flywheelSubsystem;
    }

    @Override
    public void initialize() {
        flywheelSubsystem.increasePreset();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
