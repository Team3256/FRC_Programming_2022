package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import static frc.robot.Constants.ShooterConstants.*;

public class StartShooterDefaultCommand extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public StartShooterDefaultCommand(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.setSpeed(SHOOTER_INITIAL_SPEED);
    }

    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            flywheelSubsystem.stop();
        }
    }

    @Override
    public boolean isFinished() {
        return flywheelSubsystem.isAtSetPoint();
    }
}