package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.FlywheelSubsystem;

import static frc.robot.helper.Limelight.*;

public class SimpleAutoAimShooter extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;

    public SimpleAutoAimShooter(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        Limelight.enable();
    }

    @Override
    public void execute() {
        double distanceToTarget = getTunedDistanceToTarget();

        flywheelSubsystem.simpleAutoAim(distanceToTarget);
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
