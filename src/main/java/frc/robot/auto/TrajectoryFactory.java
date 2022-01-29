package frc.robot.auto;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.commands.TrajectoryFollowCommand;
import frc.robot.helper.ThetaSupplier;
import frc.robot.helper.UniformThetaSupplier;
import frc.robot.subsystems.SwerveDrive;

import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import static frc.robot.Constants.AutoConstants.*;

public class TrajectoryFactory {
    public enum Direction {
        X, Y
    }

    private final SwerveDrive drive;

    public TrajectoryFactory(SwerveDrive drive) {
        this.drive = drive;
    }

    public Command createCommand(String jsonFilePath) {
        Trajectory trajectory = generateTrajectoryFromJSON(jsonFilePath);
        ThetaSupplier thetaSupplier = new UniformThetaSupplier(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }

    public Command createCommand(String jsonFilePath, ThetaSupplier thetaSupplier) {
        Trajectory trajectory = generateTrajectoryFromJSON(jsonFilePath);
        thetaSupplier.setTrajectoryDuration(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }

    public Command createCommand(double start, double end, Direction direction) {
        List<Pose2d> waypoints = createStraightWaypoints(start, end, direction);

        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        waypoints,
                        getDefaultTrajectoryConfig());

        ThetaSupplier thetaSupplier = new UniformThetaSupplier(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }

    public Command createCommand(double start, double end, Direction direction, ThetaSupplier thetaSupplier) {
        List<Pose2d> waypoints = createStraightWaypoints(start, end, direction);

        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        waypoints,
                        getDefaultTrajectoryConfig());

        thetaSupplier.setTrajectoryDuration(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }

    public Command createCommand(double start, double end, Direction direction, ThetaSupplier thetaSupplier, boolean reversed) {
        List<Pose2d> waypoints = createStraightWaypoints(start, end, direction);

        Trajectory trajectory =
                TrajectoryGenerator.generateTrajectory(
                        waypoints,
                        getDefaultTrajectoryConfig(reversed));

        thetaSupplier.setTrajectoryDuration(trajectory.getTotalTimeSeconds());
        return getCommand(trajectory, thetaSupplier);
    }

    private List<Pose2d> createStraightWaypoints(double start, double end, Direction direction) {
        List<Pose2d> waypoints = new ArrayList<>();
        if (direction == Direction.X) {
            for(double pos = start; pos <= end; pos += (end-start)/20) { // create 20 waypoints
                waypoints.add(new Pose2d(-pos, 0, new Rotation2d()));
            }
        }

        if (direction == Direction.Y) {
            for(double pos = start; pos <= end; pos += (end-start)/20) { // create 20 waypoints
                waypoints.add(new Pose2d(0, -pos, new Rotation2d()));
            }
        }
        return waypoints;
    }

    private Command getCommand(Trajectory trajectory, ThetaSupplier uniformThetaSupplier) {
        ProfiledPIDController thetaController =
                new ProfiledPIDController(
                        P_THETA_CONTROLLER, I_THETA_CONTROLLER, D_THETA_CONTROLLER, Constants.AutoConstants.THETA_CONTROLLER_CONSTRAINTS);
        thetaController.enableContinuousInput(-2 * Math.PI, 2 * Math.PI);

        return new TrajectoryFollowCommand(
                    trajectory,
                    new PIDController(P_X_CONTROLLER, I_X_CONTROLLER, D_X_CONTROLLER),
                    new PIDController(P_Y_CONTROLLER, I_Y_CONTROLLER, D_Y_CONTROLLER),
                    uniformThetaSupplier::rotationSupply,
                    thetaController,
                    drive
                );
    }

    private TrajectoryConfig getDefaultTrajectoryConfig() {
        return new TrajectoryConfig(
                Constants.AutoConstants.MAX_SPEED_CONTROLLER_METERS_PER_SECOND,
                Constants.AutoConstants.MAX_ACCELERATION_CONTROLLER_METERS_PER_SECOND_SQUARED)
                .setKinematics(this.drive.getKinematics());
    }

    private TrajectoryConfig getDefaultTrajectoryConfig(boolean reversed) {
        return new TrajectoryConfig(
                Constants.AutoConstants.MAX_SPEED_CONTROLLER_METERS_PER_SECOND,
                Constants.AutoConstants.MAX_ACCELERATION_CONTROLLER_METERS_PER_SECOND_SQUARED)
                .setReversed(reversed)
                .setKinematics(this.drive.getKinematics());
    }

    private Trajectory generateTrajectoryFromJSON(String trajectoryJSON){
        Trajectory trajectory = new Trajectory();
        try {
            Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve(trajectoryJSON);
            trajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
        } catch (IOException ex) {
            DriverStation.reportError("Unable to open trajectory: " + trajectoryJSON, ex.getStackTrace());
        }
        return trajectory;
    }
}
