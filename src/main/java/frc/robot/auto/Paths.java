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
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.AutoConstants.*;

public class Paths {
    public static Command getTrajectory1(SwerveDrive robotDrive) {
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.MAX_SPEED_CONTROLLER_METERS_PER_SECOND,
                        Constants.AutoConstants.MAX_ACCELERATION_CONTROLLER_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(robotDrive.getKinematics());

        List<Pose2d> waypoints = new ArrayList<>();
        for(int pos = 0; pos <= 30; pos++){
            waypoints.add(new Pose2d(Units.inchesToMeters(pos), 0, new Rotation2d()));
        }
//        List<Translation2d> waypoints = List.of(new Translation2d(Units.inchesToMeters(12), 0));s
        // JSONReader.ParseJSONFile("");

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory1 =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                       waypoints,
                config);

        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
                trajectory1,
                robotDrive::getPose, // Functional interface to feed supplies
                robotDrive.getKinematics(),


                // Position controllers
                new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                thetaController,
                robotDrive::setModuleStates,
                robotDrive);

        return swerveControllerCommand.andThen(() -> robotDrive.drive(new ChassisSpeeds()));
    }

    public static Command getTrajectory2(SwerveDrive robotDrive) {
        TrajectoryConfig config =
                new TrajectoryConfig(
                        Constants.AutoConstants.MAX_SPEED_CONTROLLER_METERS_PER_SECOND,
                        Constants.AutoConstants.MAX_ACCELERATION_CONTROLLER_METERS_PER_SECOND_SQUARED)
                        // Add kinematics to ensure max speed is actually obeyed
                        .setKinematics(robotDrive.getKinematics());

        List<Pose2d> waypoints = new ArrayList<>();
        for(int pos = 0; pos <= 80; pos++){
            waypoints.add(new Pose2d(Units.inchesToMeters(pos), 0, new Rotation2d()));
        }
//        List<Translation2d> waypoints = List.of(new Translation2d(Units.inchesToMeters(12), 0));s
        // JSONReader.ParseJSONFile("");

        // An example trajectory to follow.  All units in meters.
        Trajectory trajectory2 =
                TrajectoryGenerator.generateTrajectory(
                        // Start at the origin facing the +X direction
                        waypoints,
                        config);

        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-Math.PI, Math.PI);

        TrajectoryFollowCommand trajectoryFollowCommand = new TrajectoryFollowCommand(
                trajectory2,
                new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                thetaController,
                new Rotation2d(Units.degreesToRadians(180)),
                robotDrive
        );

        return trajectoryFollowCommand.andThen(() -> robotDrive.drive(new ChassisSpeeds()));
    }
//
//    public static SwerveControllerCommand getTrajectory3(SwerveDrive robotDrive) {
//        TrajectoryConfig config =
//                new TrajectoryConfig(
//                        Constants.AutoConstants.kMaxSpeedMetersPerSecond,
//                        Constants.AutoConstants.kMaxAccelerationMetersPerSecondSquared)
//                        // Add kinematics to ensure max speed is actually obeyed
//                        .setKinematics(Constants.SwerveConstants.kDriveKinematics);
//
//        List<Translation2d> waypoints = JSONReader.ParseJSONFile("");
//
//        // An example trajectory to follow.  All units in meters.
//        Trajectory trajectory3 =
//                TrajectoryGenerator.generateTrajectory(
//                        // Start at the origin facing the +X direction
//                        new Pose2d(0, 0, new Rotation2d(0)),
//                        // Pass through these two interior waypoints, making an 's' curve path
//                        waypoints,
//                        // End 200 inches straight ahead of where we started, facing forward
//                        new Pose2d(Units.inchesToMeters(200), 0, new Rotation2d(0)),
//                        config);
//
//        var thetaController =
//                new ProfiledPIDController(
//                        P_THETA_CONTROLLER, 0, 0, Constants.AutoConstants.kThetaControllerConstraints);
//        thetaController.enableContinuousInput(-Math.PI, Math.PI);
//
//        return new SwerveControllerCommand(
//            trajectory3,
//            robotDrive::getPose, // Functional interface to feed supplier
//            Constants.SwerveConstants.kDriveKinematics,
//
//            // Position controllers
//            new PIDController(Constants.AutoConstants.kPXController, 0, 0),
//            new PIDController(Constants.AutoConstants.kPYController, 0, 0),
//            thetaController,
//            robotDrive::setModuleStates,
//            robotDrive);
//    }
}
