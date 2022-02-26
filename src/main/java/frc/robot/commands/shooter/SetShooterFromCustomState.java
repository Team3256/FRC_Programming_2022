package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

public class SetShooterFromCustomState extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public SetShooterFromCustomState(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        SmartDashboard.setDefaultNumber("Shooter Custom RPM",100.0);
        SmartDashboard.setDefaultNumber("Hood Custom Position", 0.5);
    }

    @Override
    public void execute() {
        double dashboardShooterRPM = SmartDashboard.getNumber("Shooter Custom RPM", 100.0);
        double dashboardHoodPosition = SmartDashboard.getNumber("Shooter Custom RPM", 0.5);
        flywheelSubsystem.setSpeed(dashboardShooterRPM);
        flywheelSubsystem.setHoodAngle(dashboardHoodPosition);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}