package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class SetShooterFromCustomDashboardConfig extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    private HoodSubsystem hoodSubsystem;

    public SetShooterFromCustomDashboardConfig(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(flywheelSubsystem, hoodSubsystem);
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

        flywheelSubsystem.setPercentSpeed(velocity);
        hoodSubsystem.setHoodAngle(hoodAngle);
    }

    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.stopHood();
        flywheelSubsystem.setSpeed(0);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
