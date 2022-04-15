package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ShooterSubsystem;

public class OutakeShooter extends CommandBase {
    private ShooterSubsystem shooterSubsystem;

    public OutakeShooter(ShooterSubsystem flywheelSubsystem) {
        this.shooterSubsystem = flywheelSubsystem;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        shooterSubsystem.slowFlywheel();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return shooterSubsystem.didBallOutake();
    }
}