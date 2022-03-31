package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class StopShooter extends CommandBase {
    private ShooterSubsystem flywheelSubsystem;

    public StopShooter(ShooterSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}