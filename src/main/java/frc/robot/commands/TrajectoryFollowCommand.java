package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.helper.SwerveDriveController;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.Function;

import static frc.robot.Constants.AutoConstants.*;

public class TrajectoryFollowCommand extends CommandBase {
    private final Timer timer = new Timer();
    private final Trajectory trajectory;
    private final SwerveDriveController controller;
    private final Function<Double, Rotation2d> thetaFeeder;
    private final SwerveDrive driveSubsystem;
    private final double trajectoryDuration;

    public TrajectoryFollowCommand(
            Trajectory trajectory,
            PIDController xController,
            PIDController yController,
            Function<Double, Rotation2d> thetaFeeder,
            ProfiledPIDController thetaController,
            SwerveDrive driveSubsystem) {

        this.trajectory = trajectory;
        this.trajectoryDuration = trajectory.getTotalTimeSeconds();
        this.controller = new SwerveDriveController(
                xController,
                yController,
                thetaController
        );
        this.driveSubsystem = driveSubsystem;
        this.thetaFeeder = thetaFeeder;

        addRequirements(driveSubsystem);
    }

    @Override
    public void initialize() {
        RobotContainer.setCurrentTrajectory(trajectory);
        this.controller.reset();
        driveSubsystem.resetOdometry(new Pose2d());
        timer.reset();
        timer.start();
    }

    @Override
    public void execute() {
        double now = timer.get();
        Trajectory.State desired = trajectory.sample(now);
        Pose2d currentPose = driveSubsystem.getPose();
        Pose2d desiredPose = desired.poseMeters;
        double desiredLinearVelocity = desired.velocityMetersPerSecond;

        // Move to the desired rotation 3/4 of the way through the whole trajectory
        Rotation2d desiredRotation = thetaFeeder.apply(now);

        SmartDashboard.putNumber("Desired Rotation", desiredRotation.getDegrees());
        SmartDashboard.putNumber("Desired Position", Units.metersToInches(desiredPose.getX()));

        driveSubsystem.drive(controller.calculate(currentPose, desiredPose, desiredLinearVelocity, desiredRotation));
    }

    @Override
    public boolean isFinished() {
        return timer.get() >= trajectoryDuration;
    }
}
