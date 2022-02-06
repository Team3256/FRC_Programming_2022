package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class StopShooter extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public StopShooter(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        flywheelSubsystem.stop();
    }

    @Override
    public boolean isFinished() {
        return true;
    }
}