package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class SetShooterFromShooterState extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    private HoodSubsystem hoodSubsystem;
    private ShooterState shooterState;

    public SetShooterFromShooterState(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem, ShooterState shooterState) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        this.shooterState = shooterState;

        addRequirements(flywheelSubsystem, hoodSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        flywheelSubsystem.setPercentSpeed(shooterState.rpmVelocity);
        hoodSubsystem.setHoodAngle(shooterState.hoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.stop();
        hoodSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}