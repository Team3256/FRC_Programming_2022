package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class OutakeShooter extends CommandBase {
    private ShooterSubsystem flywheelSubsystem;

    public OutakeShooter(ShooterSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.slowFlywheel();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}