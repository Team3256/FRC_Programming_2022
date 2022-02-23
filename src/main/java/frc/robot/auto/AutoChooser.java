package frc.robot.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.drivetrain.DefaultDriveCommandRobotOriented;
import frc.robot.helper.ThetaSupplier;
import frc.robot.helper.UniformThetaSupplier;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;

public class AutoChooser {
    private static SendableChooser<Command> autoChooser;
    private static TrajectoryFactory trajectoryFactory;

    public static SendableChooser<Command> getDefaultChooser(SwerveDrive drive, IntakeSubsystem intakeSubsystem) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;

        autoChooser = new SendableChooser<>();

        Command doNothing = new DefaultDriveCommandRobotOriented(drive);
        autoChooser.setDefaultOption("Do Nothing", doNothing);

        ThetaSupplier straightPathThetaSupplier = new UniformThetaSupplier(Rotation2d.fromDegrees(180), 0.75);
        Command straightPath = trajectoryFactory.createCommand(0, 80, TrajectoryFactory.Direction.X, straightPathThetaSupplier);
        autoChooser.addOption("80 in forward 180 deg turn", straightPath);

        ThetaSupplier uniformThetaSupplier = new UniformThetaSupplier(Rotation2d.fromDegrees(180), 0.75); // trajectory factory adds duration
        Command intakePath = trajectoryFactory.createCommand("paths/Test-cones.wpilib.json", uniformThetaSupplier, new Pose2d(0, 3, new Rotation2d()));
        Command intakePathReverse = trajectoryFactory.createCommand("paths/Test-cones.wpilib.json", uniformThetaSupplier, new Pose2d(0, 3, new Rotation2d()));

        Command intakeCommand = intakePath.andThen(new WaitCommand(3).andThen(intakePathReverse));
        autoChooser.addOption("SPLINE + Intake", intakeCommand);

        ThetaSupplier to2ndBallThetaSupplier = new UniformThetaSupplier(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(90), 0.5);
        Command to2ndBall = trajectoryFactory.createCommand("paths/3BallAuto.wpilib.json", to2ndBallThetaSupplier, new Pose2d(8.963, 6.702, Rotation2d.fromDegrees(90)));
        autoChooser.addOption("2 Ball Auto", to2ndBall);

        Command curl = trajectoryFactory.createPathPlannerCommand("PathPlannerCurl", new Pose2d(1, 3, new Rotation2d()));
        autoChooser.addOption("PathPlanner Curl", curl);

        return autoChooser;
    }

    public static Command getCommand() {
        return autoChooser.getSelected();
    }
}
