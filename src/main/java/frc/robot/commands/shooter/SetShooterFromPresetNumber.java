package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class SetShooterFromPresetNumber extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public SetShooterFromPresetNumber(FlywheelSubsystem m_flywheelSubsytem) {
        flywheelSubsystem = m_flywheelSubsytem;

        addRequirements(m_flywheelSubsytem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.shootSelectedPreset();
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
