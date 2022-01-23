package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.commands.IntakeOff;
import frc.robot.commands.IntakeOn;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class AutoChooser {
    private static SendableChooser<Command> autoChooser;
    private static Command currentCommand;

    public static SendableChooser<Command> getDefaultChooser(SwerveDrive drive, IntakeSubsystem intakeSubsystem) {
        autoChooser = new SendableChooser<>();

        Command doNothing = new DefaultDriveCommand(drive); // dont move
        autoChooser.setDefaultOption("Do Nothing", doNothing);

        Command trajectory2 = Paths.getTrajectoryCommand2(drive);
        autoChooser.addOption("80in forward 180 deg turn", trajectory2);

        Command trajectory3 = Paths.getTrajectoryCommand3(drive);
        Command command3 = new ParallelCommandGroup(trajectory3, (new WaitCommand(1)).andThen(new IntakeOn(intakeSubsystem)).andThen(new WaitCommand(3)).andThen(new IntakeOff(intakeSubsystem)));
        autoChooser.addOption("SPLINE + Intake", command3);

        return autoChooser;
    }

    public static Command getCommand() {
        return autoChooser.getSelected();
    }
}
