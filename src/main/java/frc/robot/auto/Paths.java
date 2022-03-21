package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.*;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.shooter.SetShooterPIDVelocityFromState;
import frc.robot.commands.transfer.TransferIndexForward;
import frc.robot.helper.auto.AutoCommandMarker;
import frc.robot.helper.auto.AutoCommandRunner;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TransferSubsystem;

import java.util.List;

public class Paths {
    private static TrajectoryFactory trajectoryFactory;
    private static IntakeSubsystem intakeSubsystem;
    private static FlywheelSubsystem flywheelSubsystem;
    private static TransferSubsystem transferSubsystem;

    public static void initialize(SwerveDrive drive, IntakeSubsystem intake, FlywheelSubsystem flywheel, TransferSubsystem transfer) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;
        intakeSubsystem = intake;
        flywheelSubsystem = flywheel;
        transferSubsystem = transfer;
    }

    /* --------------------------------------------- */
    /* |          ANYWHERE: TARMAC (TAXIS)         | */
    /* --------------------------------------------- */

    public static Command get0BallTaxi() {
        Command taxiSegment = trajectoryFactory.createPathPlannerCommand(
                "1BallTaxi-StartTarmac",
                1, // max vel
                1, // max accel
                1, // thetakP
                0, // thetakI
                0 // thetakD
        );

        return taxiSegment;
    }

    public static Command get1BallTaxi() {
        Command taxiSegment = trajectoryFactory.createPathPlannerCommand(
                "1BallTaxi-StartTarmac",
                getOneBallRunner(),
                1, // max vel
                1, // max accel
                1, // thetakP
                0, // thetakI
                0 // thetakD
        );

        return taxiSegment
                .andThen(getShootCommand(3));
    }

    /* --------------------------------------------- */
    /* |         TWO BALL SIDE: MID TARMAC         | */
    /* --------------------------------------------- */

    public static Command get2BallMidTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallAuto-StartMidTarmac-2BallSide",
                MidTarmac2BallSide.get2BallRunner(),
                true
        ); // path planner commands cannot be reused so this whole statement cannot be in a function

        return twoBallTarmacSideSegment
                .andThen(getShootCommand(3)); // shoot
    }

    public static Command get4BallMidTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallAuto-StartMidTarmac-2BallSide",
                MidTarmac2BallSide.get2BallRunner(),
                true // is first segment
        );

        Command fourBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "4BallAuto-StartMidTarmac-2BallSide",
                MidTarmac2BallSide.get4BallRunner(),
                false // is first segment
        );

        return twoBallTarmacSideSegment
                .andThen(getShootCommand(3))
                .andThen(fourBallTarmacSideSegment);
    }

    /* --------------------------------------------- */
    /* |         ONE BALL SIDE: MID TARMAC         | */
    /* --------------------------------------------- */

    public static Command get2BallMidTarmac1BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallAuto-StartMidTarmac-1BallSide",
                MidTarmac1BallSide.get2BallRunner(),
                true // is first segment
        );

        return twoBallTarmacSideSegment
                .andThen(getShootCommand(3)); // shoot
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
                twoBallTarmacSideSegment
                .andThen(getShootCommand(3)); // shoot
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

        return twoBallTarmacSideSegment
                .andThen(new WaitCommand(1))
                .andThen(threeBallTarmacSideSegment)
                .andThen(new WaitCommand(0.1));
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
                "4BallSegment-StartEdgeTarmac-2BallSide",
                FarTarmac2BallSide.getFourBallRunner(),
                false // is first segment
        );


        return twoBallTarmacSideSegment
                .andThen(getShootCommand(3))
                .andThen(threeBallTarmacSideSegment)
                .andThen(getShootCommand(3))
                .andThen(fourBallTarmacSideSegment)
                .andThen(getShootCommand(3));
    }

    /* --------------------------------------------- */
    /* |            AUTO COMMAND RUNNERS           | */
    /* --------------------------------------------- */

    public static AutoCommandRunner getOneBallRunner() {
        List<AutoCommandMarker> oneBallSegmentMarkers = List.of(
                new AutoCommandMarker(new Translation2d(5.5, 4.50), getRevUpCommand())
        );

        return new AutoCommandRunner(oneBallSegmentMarkers);
    }

    private static class FarTarmac2BallSide {
        public static AutoCommandRunner getTwoBallRunner() {
            List<AutoCommandMarker> twoBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(7.63, 1.7), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(7.63, 0.45), getRevUpCommand())
            );

            return new AutoCommandRunner(twoBallSegmentMarkers);
        }

        public static AutoCommandRunner getThreeBallRunner() {
            List<AutoCommandMarker> threeBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(6.13, 2.53), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(5.16, 1.91), getRevUpCommand())
            );

            return new AutoCommandRunner(threeBallSegmentMarkers);
        }

        public static AutoCommandRunner getFourBallRunner() {
            List<AutoCommandMarker> fourBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(2.73, 2.42), new Translation2d(3, 1.48), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(3, 1.48), getRevUpCommand())
            );

            return new AutoCommandRunner(fourBallSegmentMarkers);
        }
    }

    private static class MidTarmac2BallSide {
        public static AutoCommandRunner get2BallRunner() {
            List<AutoCommandMarker> twoBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(6.29, 2.65), new Translation2d(4.01, 3.42), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(4.01, 3.42), getRevUpCommand())
            );

            return new AutoCommandRunner(twoBallSegmentMarkers);
        }

        public static AutoCommandRunner get4BallRunner() {
            List<AutoCommandMarker> fourBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(2.31, 2.05), new Translation2d(2.56, 0.91), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(2.56, 0.91), getRevUpCommand())
            );

            return new AutoCommandRunner(fourBallSegmentMarkers);
        }
    }

    private static class MidTarmac1BallSide {
        public static AutoCommandRunner get2BallRunner() {
            List<AutoCommandMarker> twoBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(5.97, 5.20), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(5.05, 6.16), getRevUpCommand())
            );

            return new AutoCommandRunner(twoBallSegmentMarkers);
        }
    }

    private static Command getShootCommand(double timeToShoot) {
        return
                new ParallelDeadlineGroup( // TODO dont be bad
                    new WaitCommand(timeToShoot),
                    new SetShooterPIDVelocityFromState(flywheelSubsystem, new ShooterState( 1300/*2290*/, 140000)), //TODO: FIX ME (TESTING)
                    new WaitCommand(timeToShoot * 0.25).andThen(
                            new InstantCommand(
                                    () -> CommandScheduler.getInstance().schedule(new TransferIndexForward(transferSubsystem))
                            )
                    )
                );
    }

    private static Command getRevUpCommand() {
        return new SetShooterPIDVelocityFromState(flywheelSubsystem, new ShooterState( 1300/*2290*/, 140000)); //TODO: FIX ME (TESTING)

    }
}








