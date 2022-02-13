package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;

import static frc.robot.helper.Limelight.*;

public class AutoAimShooter extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    //TODO: Add limelight subsystem definition

    public AutoAimShooter(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        //TODO: Add limelight subsystem initation
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void execute() {
        double distanceToTarget = getRawDistanceToTarget();

        flywheelSubsystem.autoAim(distanceToTarget);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
