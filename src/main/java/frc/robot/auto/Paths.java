package frc.robot.auto;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.PPTrajectoryFollowCommand;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.helper.auto.AutoCommandMarker;
import frc.robot.helper.auto.AutoCommandRunner;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

public class Paths {
    private static TrajectoryFactory trajectoryFactory;
    private static IntakeSubsystem intakeSubsystem;

    public static void initialize(SwerveDrive drive, IntakeSubsystem intake) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;
        intakeSubsystem = intake;
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
                1, // max vel
                1, // max accel
                1, // thetakP
                0, // thetakI
                0 // thetakD
        );

        return taxiSegment;
    }

    /* --------------------------------------------- */
    /* |           TWO BALL SIDE: TARMAC           | */
    /* --------------------------------------------- */

    public static Command get2BallStartTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallSegment-StartTarmac-2BallSide",
                getTwoBallSegmentTarmacRunner()
        ); // path planner commands cannot be reused so this whole statement cannot be in a function

        return twoBallTarmacSideSegment
                .andThen(new WaitCommand(1)); // shoot
    }

    public static Command get3BallStartTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallSegment-StartTarmac-2BallSide",
                getTwoBallSegmentTarmacRunner()
        );

        Command threeBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "3BallSegment-StartTarmac-2BallSide",
                getThreeBallSegmentTarmacRunner()
        );

        return twoBallTarmacSideSegment
                .andThen(new WaitCommand(1))
                .andThen(threeBallTarmacSideSegment)
                .andThen(new WaitCommand(0.1));
    }

    public static Command get4BallStartTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "2BallSegment-StartTarmac-2BallSide",
                getTwoBallSegmentTarmacRunner()
        );

        Command threeBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "3BallSegment-StartTarmac-2BallSide",
                getThreeBallSegmentTarmacRunner()
        );

        Command fourBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand(
                "4BallSegment-StartTarmac-2BallSide",
                getFourBallSegmentTarmacRunner()
        );


        return twoBallTarmacSideSegment
                .andThen(new WaitCommand(1.5)) // shoot
                .andThen(threeBallTarmacSideSegment)
                .andThen(new WaitCommand(0.1))
                .andThen(fourBallTarmacSideSegment)
                .andThen(new WaitCommand(0.1));

    }

    /* --------------------------------------------- */
    /* |            AUTO COMMAND RUNNERS           | */
    /* --------------------------------------------- */

    private static AutoCommandRunner  getOneBallSegmentTarmacRunner() {
        List<AutoCommandMarker> oneBallSegmentMarkers = List.of(
                new AutoCommandMarker(new Translation2d(5.22, 4.50), new WaitCommand(1))
        );

        return new AutoCommandRunner(oneBallSegmentMarkers);
    }

    private static AutoCommandRunner  getTwoBallSegmentTarmacRunner() {
        List<AutoCommandMarker> twoBallSegmentMarkers = List.of(
                new AutoCommandMarker(new Translation2d(7.63, 1), new IntakeOn(intakeSubsystem))
        );

        return new AutoCommandRunner(twoBallSegmentMarkers);
    }

    private static AutoCommandRunner getThreeBallSegmentTarmacRunner() {
        List<AutoCommandMarker> threeBallSegmentMarkers = List.of(
                new AutoCommandMarker(new Translation2d(5.81, 2.38), new IntakeOn(intakeSubsystem)),
                new AutoCommandMarker(new Translation2d(5.45, 2.13), new WaitCommand(1))
        );

        return new AutoCommandRunner(threeBallSegmentMarkers);
    }

    private static AutoCommandRunner getFourBallSegmentTarmacRunner() {
        List<AutoCommandMarker> fourBallSegmentMarkers = List.of(
                new AutoCommandMarker(new Translation2d(2.06, 1.42), new Translation2d(2.06, 1.42), new IntakeOn(intakeSubsystem)),
                new AutoCommandMarker(new Translation2d(5.27, 2.25), new WaitCommand(1))
        );

        return new AutoCommandRunner(fourBallSegmentMarkers);
    }
}
