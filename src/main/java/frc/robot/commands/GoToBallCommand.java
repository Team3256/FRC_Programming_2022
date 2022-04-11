package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.BallTracker;
import frc.robot.subsystems.SwerveDrive;

public class GoToBallCommand extends CommandBase {
    SwerveDrive drivetrainSubsystem;
    public GoToBallCommand(SwerveDrive drivetrainSubsystem){
        addRequirements(drivetrainSubsystem);
        this.drivetrainSubsystem=drivetrainSubsystem;
    }

    @Override
    public void execute(){
        drivetrainSubsystem.drive(new ChassisSpeeds(BallTracker.getDx()*BallTracker.getDy(), BallTracker.getDy(), 0));
    }

    @Override
    public boolean isFinished() {
        return Math.abs(BallTracker.getDx())<dxThresh &&
                Math.abs(BallTracker.getDy())<dyThresh;
    }

    double dxThresh = 0.000001;
    double dyThresh = 0.000001;
}
