package frc.robot.commands.shooter.Legacy;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.FlywheelSubsystem;

import static frc.robot.hardware.Limelight.getRawDistanceToTarget;

public class AutoPresetSelectorShooter extends CommandBase {
    /*
    private FlywheelSubsystem flywheelSubsystem;

    public AutoPresetSelectorShooter(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void initialize() {
        Limelight.enable();
    }

    @Override
    public void execute() {
        double distanceToTarget = getRawDistanceToTarget();

        flywheelSubsystem.autoPresetAutoAim(distanceToTarget);
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.disable();
    }

    @Override
    public boolean isFinished() {
        return false;
    }

     */
}

