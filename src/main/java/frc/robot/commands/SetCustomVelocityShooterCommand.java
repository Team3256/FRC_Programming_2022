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

        double theta;

        double aimHeight = UPPER_HUB_HEIGHT + PREFERRED_DISTANCE_FROM_TOP;
        double deltaHeight = aimHeight - SHOOTER_HEIGHT;

        double correctedDistance = distanceToTarget + (distanceToTarget * DELTA_DISTANCE_TO_TARGET_FACTOR);
        double correctedAimHeight = aimHeight + (aimHeight * DELTA_AIM_HEIGHT_FACTOR);

        theta = Math.atan((correctedAimHeight + UPPER_HUB_HEIGHT)/(0.5 * correctedDistance));
        flywheelSubsystem.setSpeed((correctedDistance * Math.sqrt(CONSTANT_GRAVITY) * 1/(Math.cos(theta))) / ((Math.sqrt(correctedDistance * Math.tan(theta)) - deltaHeight)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
