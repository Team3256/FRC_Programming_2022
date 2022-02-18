package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class CustomDashboardShooterSettings extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public CustomDashboardShooterSettings(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.setDefaultNumber("Custom Velocity", 100.0);
        SmartDashboard.setDefaultNumber("Custom Hood Angle", 0.0);
        SmartDashboard.setDefaultBoolean("Percent (Check) / RPM (Blank)", true);
    }

    @Override
    public void execute() {
        var velocity = SmartDashboard.getNumber("Custom Velocity", 100.0);
        var hoodAngle = SmartDashboard.getNumber("Custom Hood Angle", 0.0);

        if (SmartDashboard.getBoolean("Percent (Check) / RPM (Blank)", true)) {
            flywheelSubsystem.setPercentSpeed(velocity);
            flywheelSubsystem.setHoodAngle(hoodAngle);
        } else {
            flywheelSubsystem.setSpeed(velocity);
            flywheelSubsystem.setHoodAngle(velocity);
        }
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
