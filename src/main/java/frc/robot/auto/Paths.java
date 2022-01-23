package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.TrajectoryFollowCommand;
import frc.robot.helper.UniformThetaSupplier;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.auto.paths.JSONReader;

import java.io.IOException;
import java.lang.reflect.Array;
import java.nio.file.Path;
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

    private static Trajectory generateTrajectory(String trajectoryJSON){
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return trajectory;
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

        UniformThetaSupplier uniformThetaSupplier = new UniformThetaSupplier(trajectory.getTotalTimeSeconds(), new Rotation2d(Units.degreesToRadians(180)), 0.75);

        return getCommand(robotDrive, trajectory, uniformThetaSupplier);
    }

    public static Command getTrajectoryCommand3(SwerveDrive robotDrive) {
        String trajectoryJSON = "paths/Test.wpilib.json";
        Trajectory trajectory = generateTrajectory(trajectoryJSON);
        UniformThetaSupplier uniformThetaSupplier = new UniformThetaSupplier(trajectory.getTotalTimeSeconds(), Rotation2d.fromDegrees(-90), 0.5);

        return getCommand(robotDrive, trajectory, uniformThetaSupplier);
    }

    public static Command getTrajectoryCommand4(SwerveDrive robotDrive) {
        TrajectoryConfig config = getDefaultTrajectoryConfig(robotDrive).setReversed(true);

        List<Pose2d> waypoints = new ArrayList<>();
        for(double pos = 0; pos <= 2; pos += 0.1){
            waypoints.add(new Pose2d(-pos, 0, new Rotation2d()));
        }

        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        waypoints,
                        config);

        UniformThetaSupplier uniformThetaSupplier = new UniformThetaSupplier(trajectory.getTotalTimeSeconds(), new Rotation2d(Units.degreesToRadians(90)), 0.75);

        return getCommand(robotDrive, trajectory, uniformThetaSupplier);
    }


    private static Command getCommand(SwerveDrive robotDrive, Trajectory trajectory, UniformThetaSupplier uniformThetaSupplier ) {
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-2 * Math.PI, 2 * Math.PI);

        TrajectoryFollowCommand trajectoryFollowCommand = new TrajectoryFollowCommand(
                trajectory,
                new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                uniformThetaSupplier::rotationSupply,
                thetaController,
                robotDrive
        );

        return trajectoryFollowCommand;
    }
}