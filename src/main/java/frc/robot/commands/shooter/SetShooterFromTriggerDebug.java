package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

import java.util.function.DoubleSupplier;

public class SetShooterFromTriggerDebug extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    private DoubleSupplier triggerInput;

    public SetShooterFromTriggerDebug(FlywheelSubsystem flywheelSubsystem, DoubleSupplier triggerInput) {
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
        flywheelSubsystem.stopFlywheel();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
