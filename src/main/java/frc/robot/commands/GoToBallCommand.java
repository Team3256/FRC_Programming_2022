package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

public class GoToBallCommand extends CommandBase {
    SwerveDrive drivetrainSubsystem;
    public GoToBallCommand(SwerveDrive drivetrainSubsystem){
        addRequirements(drivetrainSubsystem);
        this.drivetrainSubsystem=drivetrainSubsystem;
    }

    
}
