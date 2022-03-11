package frc.robot.commands.shooter;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.FlywheelSubsystem;

import java.util.function.DoubleConsumer;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.ALL_SHOOTER_PRESETS;


public class ShooterPIDCommand extends CommandBase {
    private FlywheelSubsystem.ShooterLocationPreset shooterLocationPreset;

    private FlywheelSubsystem flywheelSubsystem;
    public ShooterPIDCommand(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.shooterLocationPreset = flywheelSubsystem.getShooterLocationPreset();
    }

    @Override
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        super.execute();
    }

    @Override
    public void end(boolean interrupted) {
        super.end(interrupted);
    }
}
