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
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        String name = flywheelSubsystem.increasePreset();
        Logger logger = Logger.getLogger(Robot.class.getCanonicalName());
        logger.info("Current Preset: " + name);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            flywheelSubsystem.stopFlywheel();
            flywheelSubsystem.stopHood();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
