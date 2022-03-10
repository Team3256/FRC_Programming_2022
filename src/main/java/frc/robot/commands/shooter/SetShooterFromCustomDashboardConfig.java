package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class SetShooterFromCustomDashboardConfig extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public SetShooterFromCustomDashboardConfig(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.setDefaultNumber("Custom Velocity", 1);
        SmartDashboard.setDefaultNumber("Custom Hood Angle", 0.0);
    }

    @Override
    public void execute() {
        double velocity = SmartDashboard.getNumber("Custom Velocity", 1);
        double hoodAngle = SmartDashboard.getNumber("Custom Hood Angle", 0.0);

        flywheelSubsystem.setSpeed(velocity);
        flywheelSubsystem.setHoodAngle(hoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        flywheelSubsystem.stopFullShooter();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
