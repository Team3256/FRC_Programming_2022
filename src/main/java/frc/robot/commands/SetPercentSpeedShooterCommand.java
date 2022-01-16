package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

import java.util.function.DoubleSupplier;

public class SetPercentSpeedShooterCommand extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    private DoubleSupplier triggerInput;

    public SetPercentSpeedShooterCommand(FlywheelSubsystem flywheelSubsystem, DoubleSupplier triggerInput) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.triggerInput = triggerInput;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void execute() {
        flywheelSubsystem.setPercentSpeed(triggerInput.getAsDouble() * 100);
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
