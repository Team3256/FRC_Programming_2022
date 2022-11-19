package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.drivetrain.DefaultDriveCommandRobotOriented;
import frc.robot.commands.shooter.ZeroHoodMotorCommand;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TransferSubsystem;

public class AutoChooser {
    private static SendableChooser<Command> autoChooser;
    private static TrajectoryFactory trajectoryFactory;
    private static ShooterSubsystem flywheelSubsystem;

    public static SendableChooser<Command> getDefaultChooser(SwerveDrive drive, IntakeSubsystem intake, ShooterSubsystem flywheel, TransferSubsystem transfer) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;
        flywheelSubsystem = flywheel;
        Paths.initialize(drive, intake, flywheel, transfer);

        autoChooser = new SendableChooser<>();

        if (drive != null) {
            Command doNothing = new DefaultDriveCommandRobotOriented(drive);
            autoChooser.setDefaultOption("Do Nothing", doNothing);

            // path planner dot is the shooter
            if (flywheel != null) {
                if (intake != null && transfer != null) {
                    // TODO: Add your command to the chooser

                    Command oneBallPicked = Paths.getPleasePickUsOneBall();
                    autoChooser.addOption("JUST ONE BALL", oneBallPicked);

                    Command oneBallPickedHanger = Paths.getPleasePickUsHanger();
                    autoChooser.addOption("HANGER DEFENSE", oneBallPickedHanger);

                    Command oneBallPickedFender = Paths.getPleasePickUsFender();
                    autoChooser.addOption("FENDER DEFENSE", oneBallPickedFender);

                    Command cool = Paths.getCoolAuto();
                    autoChooser.addOption("COOL AUTO RUN THIS", cool);

                    Command twoBallTarmacEdge2BallSide = Paths.get2BallFarTarmac2BallSide();
                    autoChooser.addOption("2 Ball | Start Edge Tarmac | 2 Ball Side", twoBallTarmacEdge2BallSide);

                    Command threeBallTarmacEdge2BallSide = Paths.get3BallFarTarmac2BallSide();
                    autoChooser.addOption("3 Ball | Start Edge Tarmac | 2 Ball Side", threeBallTarmacEdge2BallSide);

                    Command fourBallTarmacEdge2BallSide = Paths.get4BallFarTarmac2BallSide();
                    autoChooser.addOption("4/5 Ball | Start Edge Tarmac | 2 Ball Side", fourBallTarmacEdge2BallSide);
                }
            }
        }

        return autoChooser;
    }

    public static Command getCommand() {
        return flywheelSubsystem != null ?
                new ParallelRaceGroup(
                    new WaitCommand(3),
                    new ZeroHoodMotorCommand(flywheelSubsystem)
                ).andThen(autoChooser.getSelected())
                :
            autoChooser.getSelected();
    }
}
