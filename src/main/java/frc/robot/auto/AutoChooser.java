package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class AutoChooser {
    private static SendableChooser<Command> autoChooser;
    private static Command currentCommand;

    public static SendableChooser<Command> getDefaultChooser(SwerveDrive drive) {
        autoChooser = new SendableChooser<>();

        Command doNothing = new DefaultDriveCommand(drive); // dont move
        autoChooser.setDefaultOption("Do Nothing", doNothing);

        Command trajectory2 = Paths.getTrajectoryCommand2(drive);
        autoChooser.addOption("80in forward 180 deg turn", trajectory2);

        Command trajectory3 = Paths.getTrajectoryCommand2(drive);
        autoChooser.addOption("Dont use", trajectory3);

        return autoChooser;
    }

    public static Command getCommand() {
        return autoChooser.getSelected();
    }
}
