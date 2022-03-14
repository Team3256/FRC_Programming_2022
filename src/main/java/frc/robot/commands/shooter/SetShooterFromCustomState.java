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
    }

    @Override
    public void execute() {
        flywheelSubsystem.setPercentSpeed(0.7);
        flywheelSubsystem.setHoodAngle(20000);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}