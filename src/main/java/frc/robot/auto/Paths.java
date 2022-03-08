package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.shooter.AutoPresetSelectorShooter;
import frc.robot.commands.transfer.TransferIndexForward;
import frc.robot.helper.auto.AutoCommandMarker;
import frc.robot.helper.auto.AutoCommandRunner;
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

        return taxiSegment
                .andThen(new WaitCommand(0.1));
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
                .andThen(getShootCommand(1.5))
                .andThen(fourBallTarmacSideSegment)
                .andThen(new WaitCommand(0.1));
    }

    /* --------------------------------------------- */
    /* |         TWO BALL SIDE: FAR TARMAC         | */
    /* --------------------------------------------- */

    public static Command get2BallFarTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallSegment-StartEdgeTarmac-2BallSide",
                FarTarmac2BallSide.getTwoBallRunner(),
                true // is first segment
        ); // path planner commands cannot be reused so this whole statement cannot be in a function

        return twoBallTarmacSideSegment
                .andThen(getShootCommand(3)); // shoot
    }

    public static Command get3BallFarTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallSegment-StartEdgeTarmac-2BallSide",
                FarTarmac2BallSide.getThreeBallRunner(),
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
                .andThen(getShootCommand(1.5))
                .andThen(threeBallTarmacSideSegment)
                .andThen(getShootCommand(1.5))
                .andThen(fourBallTarmacSideSegment)
                .andThen(getShootCommand(1.5));
    }

    /* --------------------------------------------- */
    /* |            AUTO COMMAND RUNNERS           | */
    /* --------------------------------------------- */

    public static AutoCommandRunner getOneBallRunner() {
        List<AutoCommandMarker> oneBallSegmentMarkers = List.of(
                new AutoCommandMarker(new Translation2d(5.22, 4.50), getShootCommand(3))
        );

        return new AutoCommandRunner(oneBallSegmentMarkers);
    }

    private static class FarTarmac2BallSide {
        public static AutoCommandRunner getTwoBallRunner() {
            List<AutoCommandMarker> twoBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(7.63, 1), new IntakeOn(intakeSubsystem))
            );

            return new AutoCommandRunner(twoBallSegmentMarkers);
        }

        public static AutoCommandRunner getThreeBallRunner() {
            List<AutoCommandMarker> threeBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(5.81, 2.38), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(5.45, 2.13), getShootCommand(3))
            );

            return new AutoCommandRunner(threeBallSegmentMarkers);
        }

        public static AutoCommandRunner getFourBallRunner() {
            List<AutoCommandMarker> fourBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(2.06, 1.42), new Translation2d(2.06, 1.42), new IntakeOn(intakeSubsystem)),
                    new AutoCommandMarker(new Translation2d(5.27, 2.25), getShootCommand(3))
            );

            return new AutoCommandRunner(fourBallSegmentMarkers);
        }
    }

    private static class MidTarmac2BallSide {
        public static AutoCommandRunner get2BallRunner() {
            List<AutoCommandMarker> twoBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(5.45, 2.14), new Translation2d(5.10, 2.49), new IntakeOn(intakeSubsystem))
            );

            return new AutoCommandRunner(twoBallSegmentMarkers);
        }

        public static AutoCommandRunner get4BallRunner() {
            List<AutoCommandMarker> fourBallSegmentMarkers = List.of(
                    new AutoCommandMarker(new Translation2d(1.73, 1.42), new Translation2d(1.42, 1.08), new IntakeOn(intakeSubsystem))
            );

            return new AutoCommandRunner(fourBallSegmentMarkers);
        }
    }

    private static Command getShootCommand(double timeToShoot) {
        return
                new ParallelDeadlineGroup(
                    new WaitCommand(timeToShoot),
                    new AutoPresetSelectorShooter(flywheelSubsystem),
                    new TransferIndexForward(transferSubsystem)
                );
    }
}