package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

public class SetShooterFromCustomState extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    private HoodSubsystem hoodSubsystem;

    public SetShooterFromCustomState(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.hoodSubsystem = hoodSubsystem;

        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
        flywheelSubsystem.setPercentSpeed(0.60);
        hoodSubsystem.setHoodAngle(100000);
    }

    @Override
    public void end(boolean interrupted) {
        hoodSubsystem.stopHood();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}