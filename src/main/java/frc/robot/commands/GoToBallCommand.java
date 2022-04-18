package frc.robot.commands;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.BallTracker;
import frc.robot.subsystems.SwerveDrive;

import static frc.robot.Constants.BallFollowConstants.*;

public class GoToBallCommand extends CommandBase {
    SwerveDrive drivetrainSubsystem;
    public GoToBallCommand(SwerveDrive drivetrainSubsystem){
        addRequirements(drivetrainSubsystem);
        this.drivetrainSubsystem=drivetrainSubsystem;
    }

    /**
     * swerve drives towards ball
     */
    @Override
    public void execute(){
        drivetrainSubsystem.drive(new ChassisSpeeds(KX * BallTracker.getDx()*BallTracker.getDy(), KY * BallTracker.getDy(), 0));
    }

    /**
     * @return is position relative to ball is less than error
     */
    @Override
    public boolean isFinished() {
        return Math.abs(BallTracker.getDx())<DX_MAX_ERROR &&
                Math.abs(BallTracker.getDy())<DY_MAX_ERROR;
    }

    /**
     * stop robot when done
     * @param interrupted
     */
    @Override
    public void end(boolean interrupted){
        drivetrainSubsystem.stop();
    }
}
