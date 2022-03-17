package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Limelight;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.HoodSubsystem;

import static frc.robot.hardware.Limelight.getRawDistanceToTarget;

public class SimpleAutoAimShooter extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    private HoodSubsystem hoodSubsystem;

    public SimpleAutoAimShooter(FlywheelSubsystem flywheelSubsystem, HoodSubsystem hoodSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        this.hoodSubsystem = hoodSubsystem;
        addRequirements(flywheelSubsystem, hoodSubsystem);
    }

    @Override
    public void initialize() {
        Limelight.enable();
    }

    @Override
    public void execute() {
        double distanceToTarget = getRawDistanceToTarget();
        flywheelSubsystem.setSpeed(flywheelSubsystem.getFlywheelRPMFromInterpolator(distanceToTarget));
        hoodSubsystem.setHoodAngle(hoodSubsystem.getHoodAngleFromInterpolator(distanceToTarget));
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
