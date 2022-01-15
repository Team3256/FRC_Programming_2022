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
    public void initialize() {
        super.initialize();
    }

    @Override
    public void execute() {
        double distanceToTarget = 0.0; //TODO: limelight fetch distance goes here

        double theta;

        double aimHeight = UPPER_HUB_HEIGHT - PREFERRED_DISTANCE_FROM_TOP;
        double deltaHeight = aimHeight - SHOOTER_HEIGHT;

        theta = Math.atan((aimHeight + UPPER_HUB_HEIGHT)/(0.5 * distanceToTarget));
        flywheelSubsystem.setSpeed((distanceToTarget * Math.sqrt(CONSTANT_GRAVITY) * 1/(Math.cos(theta))) / ((Math.sqrt(distanceToTarget * Math.tan(theta)) - deltaHeight)));
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
