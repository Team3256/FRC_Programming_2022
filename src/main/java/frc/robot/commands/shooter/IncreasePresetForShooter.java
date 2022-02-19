package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class IncreasePresetForShooter extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public IncreasePresetForShooter(FlywheelSubsystem m_flywheelSubsystem) {
        flywheelSubsystem = m_flywheelSubsystem;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        double distance = flywheelSubsystem.increasePreset();
        SmartDashboard.putNumber("Distance to Target (Preset)", distance);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            flywheelSubsystem.stop();
            flywheelSubsystem.stopHood();
        }
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}
