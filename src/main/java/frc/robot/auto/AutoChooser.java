package frc.robot.auto;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeOff;
import frc.robot.commands.IntakeOn;
import frc.robot.helper.ThetaSupplier;
import frc.robot.helper.UniformThetaSupplier;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;

public class AutoChooser {
    private static SendableChooser<Command> autoChooser;
    private static TrajectoryFactory trajectoryFactory;

    public static SendableChooser<Command> getDefaultChooser(SwerveDrive drive, IntakeSubsystem intakeSubsystem) {
        trajectoryFactory= trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;

        autoChooser = new SendableChooser<>();

        Command doNothing = new DefaultDriveCommand(drive); // dont move
        autoChooser.setDefaultOption("Do Nothing", doNothing);

        ThetaSupplier straightPathThetaSupplier = new UniformThetaSupplier(Rotation2d.fromDegrees(180), 0.75);
        Command straightPath = trajectoryFactory.createCommand(0, 80, TrajectoryFactory.Direction.X, straightPathThetaSupplier);
        autoChooser.addOption("80in forward 180 deg turn", straightPath);

        ThetaSupplier uniformThetaSupplier = new UniformThetaSupplier(Rotation2d.fromDegrees(-90), 0.5); // trajectory factory adds duration
        Command intakePath = trajectoryFactory.createCommand("paths/Test.wpilib.json", uniformThetaSupplier);

        Command intakeCommamd = new ParallelDeadlineGroup(
                intakePath,
                (new WaitCommand(1))
                        .andThen(new IntakeOn(intakeSubsystem))
                        .andThen(new WaitCommand(3))
                        .andThen(new IntakeOff(intakeSubsystem))
                );

        autoChooser.addOption("SPLINE + Intake", intakeCommamd);

        return autoChooser;
    }

    public static Command getCommand() {
        return autoChooser.getSelected();
    }
}
