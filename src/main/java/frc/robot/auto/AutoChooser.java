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
        Paths.initialize(drive);

        autoChooser = new SendableChooser<>();

        Command doNothing = new DefaultDriveCommandRobotOriented(drive);
        autoChooser.setDefaultOption("Do Nothing", doNothing);

        Command twoBallTarmac2BallSide = Paths.get2BallStartTarmac2BallSide();
        autoChooser.addOption("2 Ball | Start Tarmac | 2 Ball Side", twoBallTarmac2BallSide);

        Command threeBallTarmac2BallSide = Paths.get3BallStartTarmac2BallSide();
        autoChooser.addOption("3 Ball | Start Tarmac | 2 Ball Side", threeBallTarmac2BallSide);

        Command fourBallTarmac2BallSide = Paths.get4BallStartTarmac2BallSide();
        autoChooser.addOption("4 Ball | Start Tarmac | 2 Ball Side", fourBallTarmac2BallSide);

        return autoChooser;
    }

    public static Command getCommand() {
        return autoChooser.getSelected();
    }
}
