package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.drivetrain.AutoAlignDriveCommand;
import frc.robot.commands.drivetrain.DefaultDriveCommandRobotOriented;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.intake.IntakeReverse;
import frc.robot.commands.shooter.SetShooterPIDFromInterpolation;
import frc.robot.commands.transfer.OuttakeFast;
import frc.robot.commands.transfer.TransferManualReverse;
import frc.robot.commands.transfer.TransferShootForward;
import frc.robot.hardware.Limelight;
import frc.robot.helper.auto.AutoCommandMarker;
import frc.robot.helper.auto.AutoCommandRunner;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TransferSubsystem;

import java.util.List;

import static frc.robot.Constants.AutoConstants.*;

public class Paths {
    private static SwerveDrive driveSubsystem;
    private static TrajectoryFactory trajectoryFactory;
    private static IntakeSubsystem intakeSubsystem;
    private static ShooterSubsystem shooterSubsystem;
    private static TransferSubsystem transferSubsystem;

    public static void initialize(SwerveDrive drive, IntakeSubsystem intake, ShooterSubsystem shooter, TransferSubsystem transfer) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;
        driveSubsystem = drive;
        intakeSubsystem = intake;
        shooterSubsystem = shooter;
        transferSubsystem = transfer;
    }

    // TODO: Implement this
    public static Command getSymbol5Command() {
        Command Symbol5Command = trajectoryFactory.createPathPlannerCommand("Symbol5");
        return Symbol5Command;
    }

    /* --------------------------------------------- */
    /* |            EXAMPLE AUTOS BELOW            | */
    /* --------------------------------------------- */

    /* --------------------------------------------- */
    /* |         1619 & 254 PLEASE PICK US         | */
    /* --------------------------------------------- */

    public static Command getPleasePickUsOneBall() {
        Command oneBallSegment = trajectoryFactory.createPathPlannerCommand(
                "1BallAutoDefenseStart-StartMidTarmac-1BallSide",
                PleasePickUs.getOneBallRunner(),
                13,
                5,
                5.5,
                I_THETA_CONTROLLER,
                D_THETA_CONTROLLER
        );

        return oneBallSegment
                .andThen(getShootCommand(4));
    }

    public static Command getPleasePickUsHanger() {
        Command oneBallSegment = trajectoryFactory.createPathPlannerCommand(
                "1BallAutoDefenseStart-StartMidTarmac-1BallSide",
                PleasePickUs.getOneBallRunner(),
                13,
                5,
                5.5,
                I_THETA_CONTROLLER,
                D_THETA_CONTROLLER
        );

        Command hangerOuttakeSegment = trajectoryFactory.createPathPlannerCommand(
                "1BallAutoHanger-StartMidTarmac-1BallSide",
                PleasePickUs.getHangerRunner(),
                13,
                5,
                5.5,
                I_THETA_CONTROLLER,
                D_THETA_CONTROLLER,
                false
        );

        return oneBallSegment
                .andThen(getShootCommand(4))
                .andThen(hangerOuttakeSegment)
                .andThen(new InstantCommand(() -> CommandScheduler.getInstance().schedule(new OuttakeFast(transferSubsystem, intakeSubsystem))));

    }

    public static Command getPleasePickUsFender() {
        Command oneBallSegment = trajectoryFactory.createPathPlannerCommand(
                "1BallAutoDefenseStart-StartMidTarmac-1BallSide",
                PleasePickUs.getOneBallRunner(),
                13,
                5,
                5.5,
                I_THETA_CONTROLLER,
                D_THETA_CONTROLLER
        );

        Command fenderOuttakeSegment = trajectoryFactory.createPathPlannerCommand(
                "1BallAutoFender-StartMidTarmac-1BallSide",
                PleasePickUs.getFenderRunner(),
                13,
                3,
                5.5,
                I_THETA_CONTROLLER,
                D_THETA_CONTROLLER,
                false
        );


        Command outtakeSlow = new ParallelCommandGroup(
                new TransferManualReverse(transferSubsystem),
                new IntakeReverse(intakeSubsystem)
        );

        return oneBallSegment
                .andThen(getShootCommand(4))
                .andThen(fenderOuttakeSegment)
                .andThen(new InstantCommand(() -> CommandScheduler.getInstance().schedule(outtakeSlow)))
                .andThen(new WaitCommand(5))
                .andThen(new InstantCommand(() -> CommandScheduler.getInstance().cancel(outtakeSlow)));

    }

    /* --------------------------------------------- */
    /* |         TWO BALL SIDE: FAR TARMAC         | */
    /* --------------------------------------------- */

    public static Command get2BallFarTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallSegment-StartEdgeTarmac-2BallSide",
                FarTarmac2BallSide.getTwoBallRunner()
        ); // path planner commands cannot be reused so this whole statement cannot be in a function

        return
            getShootCommand(3)
                .andThen(twoBallTarmacSideSegment)
                .andThen(getShootCommand(3)); // shoot
    }

    public static Command getCoolAuto() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallSegment-StartEdgeTarmac-2BallSide",
                FarTarmac2BallSide.getTwoBallRunner(),
                true
        ); // path planner commands cannot be reused so this whole statement cannot be in a function


        Command coolSegment = trajectoryFactory.createPathPlannerCommand(
                "CoolAutoForSid",
                new AutoCommandRunner(List.of(new AutoCommandMarker(new Translation2d(7.25, 1.31), new IntakeOn(intakeSubsystem)))),
                false
        );

        Command out = new ParallelCommandGroup(
                new TransferManualReverse(transferSubsystem),
                new IntakeReverse(intakeSubsystem)
        );

        return 
            getShootCommand(3)
                .andThen(twoBallTarmacSideSegment)
                .andThen(getShootCommand(4))
                .andThen(coolSegment)
                .andThen(new InstantCommand(() -> CommandScheduler.getInstance().schedule(out)));
    }

    public static Command get3BallFarTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallSegment-StartEdgeTarmac-2BallSide",
                FarTarmac2BallSide.getTwoBallRunner(),
                true // is first segment
        );

        Command threeBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "3BallSegment-StartEdgeTarmac-2BallSide",
                FarTarmac2BallSide.getThreeBallRunner(),
                false // is first segment
        );

        return 
            getShootCommand(3)
                .andThen(twoBallTarmacSideSegment)
                .andThen(getShootCommand(3))
                .andThen(threeBallTarmacSideSegment)
                .andThen(getShootCommand(3));
    }

    public static Command get4BallFarTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallSegment-StartEdgeTarmac-2BallSide",
                FarTarmac2BallSide.getTwoBallRunner(),
                true // is first segment
        );

        Command threeBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "3BallSegment-StartEdgeTarmac-2BallSide",
                FarTarmac2BallSide.getThreeBallRunner(),
                false // is first segment
        );

        Command fourBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "4BallAuto-StartTarmac-2BallSide",
                FarTarmac2BallSide.getFourBallRunner(),
                false // is first segment
        );


        return 
            getShootCommand(3)
                .andThen(twoBallTarmacSideSegment)
                .andThen(getShootCommand(4.5))
                .andThen(threeBallTarmacSideSegment)
                .andThen(getShootCommand(3))
                .andThen(fourBallTarmacSideSegment)
                .andThen(getShootCommand(3));
    }

    /* --------------------------------------------- */
    /* |            AUTO COMMAND RUNNERS           | */
    /* --------------------------------------------- */

    private static class PleasePickUs {
        public static AutoCommandRunner getOneBallRunner() {
            List<AutoCommandMarker> oneBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(5.85, 4.33), getRevUpCommand())
            );

            return new AutoCommandRunner(oneBallSegmentMarkers);
        }

        public static AutoCommandRunner getHangerRunner() {
            List<AutoCommandMarker> oneBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(4.65, 4.33), new Translation2d(4.43, 3.32), new IntakeOn(intakeSubsystem))
            );

            return new AutoCommandRunner(oneBallSegmentMarkers);
        }

        public static AutoCommandRunner getFenderRunner() {
            List<AutoCommandMarker> oneBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(4.65, 4.33), new Translation2d(4.55, 3.32), new IntakeOn(intakeSubsystem))
            );

            return new AutoCommandRunner(oneBallSegmentMarkers);
        }
    }

    public static AutoCommandRunner getOneBallRunner() {
        List<AutoCommandMarker> oneBallSegmentMarkers = List.of(
                new AutoCommandMarker(new Translation2d(5.83, 4.50), getRevUpCommand())
        );

        return new AutoCommandRunner(oneBallSegmentMarkers);
    }

    private static class FarTarmac2BallSide {
        public static AutoCommandRunner getTwoBallRunner() {
            List<AutoCommandMarker> twoBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(7.63, 1.7), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(7.63, 1.7), getRevUpCommand())
            );

            return new AutoCommandRunner(twoBallSegmentMarkers);
        }

        public static AutoCommandRunner getThreeBallRunner() {
            List<AutoCommandMarker> threeBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(5.91, 2.52), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(5.91, 2.52), getRevUpCommand())
            );

            return new AutoCommandRunner(threeBallSegmentMarkers);
        }

        public static AutoCommandRunner getFourBallRunner() {
            List<AutoCommandMarker> fourBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(5.16, 1.91), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(2.50, 1.97), getRevUpCommand())
            );

            return new AutoCommandRunner(fourBallSegmentMarkers);
        }
    }

    private static Command getShootCommand(double timeToShoot) {
        double now = Timer.getFPGATimestamp();
        Command transferShootForward = new TransferShootForward(transferSubsystem, shooterSubsystem, () -> Timer.getFPGATimestamp() - now > 3);
        return
                new ParallelDeadlineGroup( // TODO dont be bad
                    new WaitCommand(timeToShoot * 0.7),
                    new AutoAlignDriveCommand(driveSubsystem),
                    new SetShooterPIDFromInterpolation(shooterSubsystem, driveSubsystem::getEstimatedDistance, Limelight::isTargetDetected),
//                    new SetShooterPIDVelocityFromState(flywheelSubsystem, ()->new ShooterState( 2450, 140000)), //TODO: FIX ME (TESTING)
                    new WaitCommand(timeToShoot * 0.20).andThen(
                            new InstantCommand(
                                    () -> CommandScheduler.getInstance().schedule(transferShootForward)
                            )
                    )
                ).andThen(new InstantCommand(
                        () -> CommandScheduler.getInstance().cancel(transferShootForward)
                ));
    }

    private static Command getRevUpCommand() {
//        return new SetShooterPIDVelocityFromState(shooterSubsystem, ()->new ShooterState( 2450, 140000)); //TODO: FIX ME (TESTING)
        return new SetShooterPIDFromInterpolation(shooterSubsystem, driveSubsystem::getEstimatedDistance, Limelight::isTargetDetected);
    }
}








