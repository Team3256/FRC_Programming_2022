package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.commands.TrajectoryFollowCommand;
import frc.robot.helper.UniformThetaSupplier;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.AutoConstants.*;

public class Paths {
    private static TrajectoryConfig getDefaultTrajectoryConfig(SwerveDrive robotDrive) {
        return new TrajectoryConfig(
                        Constants.AutoConstants.MAX_SPEED_CONTROLLER_METERS_PER_SECOND,
                        Constants.AutoConstants.MAX_ACCELERATION_CONTROLLER_METERS_PER_SECOND_SQUARED)
                        .setKinematics(robotDrive.getKinematics());
    }

    public static Command getTrajectoryCommand2(SwerveDrive robotDrive) {
        TrajectoryConfig config = getDefaultTrajectoryConfig(robotDrive);

        List<Pose2d> waypoints = new ArrayList<>();
        for(int pos = 0; pos <= 80; pos++){
            waypoints.add(new Pose2d(Units.inchesToMeters(pos), 0, new Rotation2d()));
        }

        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        waypoints,
                        config);

        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        UniformThetaSupplier thetaFeed = new UniformThetaSupplier(trajectory.getTotalTimeSeconds(), new Rotation2d(Units.degreesToRadians(180)), 0.75);
        TrajectoryFollowCommand trajectoryFollowCommand = new TrajectoryFollowCommand(
                trajectory,
                new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                thetaFeed::rotationSupply,
                thetaController,
                robotDrive
        );

        return trajectoryFollowCommand.andThen(() -> robotDrive.drive(new ChassisSpeeds()));
    }
}
