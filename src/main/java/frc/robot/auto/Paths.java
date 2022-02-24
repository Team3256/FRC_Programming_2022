package frc.robot.auto;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.SwerveDrive;

public class Paths {
    private static TrajectoryFactory trajectoryFactory;

    public static void initialize(SwerveDrive drive) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;
    }

    /* --------------------------------------------- */
    /* |               ANYWHERE: TARMAC            | */
    /* --------------------------------------------- */

    public static Command get0BallTaxi() {
        Command taxiSegment = trajectoryFactory.createPathPlannerCommand("1BallTaxi-StartAnywhere");
        return taxiSegment
                .andThen(new WaitCommand(0.1));
    }

    public static Command get1BallTaxi() {
        Command taxiSegment = trajectoryFactory.createPathPlannerCommand("1BallTaxi-StartAnywhere");
        return taxiSegment
                .andThen(new WaitCommand(1.5)); // shoot
    }

    /* --------------------------------------------- */
    /* |           TWO BALL SIDE: TARMAC           | */
    /* --------------------------------------------- */

    public static Command get2BallStartTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand("2BallSegment-StartTarmac-2BallSide");

        return twoBallTarmacSideSegment
                .andThen(new WaitCommand(1)); // shoot
    }

    public static Command get3BallStartTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand("2BallSegment-StartTarmac-2BallSide");
        Command threeBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand("3BallSegment-StartTarmac-2BallSide");

        return twoBallTarmacSideSegment
                .andThen(new WaitCommand(1))
                .andThen(threeBallTarmacSideSegment)
                .andThen(new WaitCommand(0.1));
    }

    public static Command get4BallStartTarmac2BallSide() {
        Command twoBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand("2BallSegment-StartTarmac-2BallSide");
        Command threeBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand("3BallSegment-StartTarmac-2BallSide");
        Command fourBallTarmacSideSegment = trajectoryFactory.createPathPlannerCommand("4BallSegment-StartTarmac-2BallSide");

        return twoBallTarmacSideSegment
                .andThen(new WaitCommand(1.5)) // shoot
                .andThen(threeBallTarmacSideSegment)
                .andThen(new WaitCommand(0.1))
                .andThen(fourBallTarmacSideSegment)
                .andThen(new WaitCommand(0.1));

    }
}
