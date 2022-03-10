package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.shooter.ShooterPreset;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.FlywheelSubsystem.ShooterLocationPreset;

import static frc.robot.Constants.ShooterConstants.ALL_SHOOTER_PRESETS;

public class SetShooterFromLocationPreset extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    private ShooterLocationPreset shooterLocationPreset;
    private ShooterState shooterState;

    public SetShooterFromLocationPreset(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;

        addRequirements(flywheelSubsystem);
    }

    public SetShooterFromLocationPreset(FlywheelSubsystem flywheelSubsystem, ShooterLocationPreset preset) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.shooterLocationPreset = preset;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        if (shooterLocationPreset != null) {
            flywheelSubsystem.setShooterLocationPreset(shooterLocationPreset);
        }
        shooterState = ALL_SHOOTER_PRESETS.get(flywheelSubsystem.getShooterLocationPreset()).shooterState;
    }

    @Override
    public void execute() {
        flywheelSubsystem.setSpeed(shooterState.rpmVelocity);
        flywheelSubsystem.setHoodAngle(shooterState.hoodAngle);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
