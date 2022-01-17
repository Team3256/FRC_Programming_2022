package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FlywheelSubsystem;
import static frc.robot.Constants.ShooterConstants.*;

public class SetCustomVelocityShooterCommand extends CommandBase {
    private FlywheelSubsystem flywheelSubsystem;
    //TODO: Add limelight subsystem definition

    public SetCustomVelocityShooterCommand(FlywheelSubsystem flywheelSubsystem) {
        this.flywheelSubsystem = flywheelSubsystem;
        //TODO: Add limelight subsystem initation
        addRequirements(flywheelSubsystem);
    }

    @Override
    public void execute() {
        double distanceToTarget = 0.0; //TODO: limelight fetch distance goes here

        flywheelSubsystem.autoAim(distanceToTarget);
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
