package frc.robot.auto;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.drivetrain.DefaultDriveCommandRobotOriented;
import frc.robot.commands.shooter.ZeroHoodMotorCommand;
import frc.robot.helper.shooter.ShooterPreset;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.subsystems.TransferSubsystem;
import frc.robot.RobotContainer.*;

public class AutoChooser {
    private static SendableChooser<Command> autoChooser;
    private static SendableChooser<BallCount> ballCountChooser;
    private static SendableChooser<StartingPosition> startingPositionChooser;

    private static TrajectoryFactory trajectoryFactory;
    private static ShooterSubsystem flywheelSubsystem;

    public enum StartingPosition {
        TARMAC_ANY_SIDE,
        MID_TARMAC_ONE_BALL_SIDE,
        MID_TARMAC_TWO_BALL_SIDE,
        EDGE_TARMAC_TWO_BALL_SIDE,
    }

    public enum BallCount {
        ZERO_BALL,
        ONE_BALL,
        TWO_BALL,
        THREE_BALL,
        FOUR_BALL,
        FIVE_BALL
    }

    public static SendableChooser getDefaultChooser(SwerveDrive drive, IntakeSubsystem intake, ShooterSubsystem flywheel, TransferSubsystem transfer, DefaultChooserOptions option) {
        trajectoryFactory = trajectoryFactory == null ? new TrajectoryFactory(drive) : trajectoryFactory;
        flywheelSubsystem = flywheel;
        Paths.initialize(drive, intake, flywheel, transfer);

        autoChooser = new SendableChooser<>();
        ballCountChooser = new SendableChooser<>();
        startingPositionChooser = new SendableChooser<>();

        for (BallCount numberOfBalls : BallCount.values()) {
            ballCountChooser.addOption("Auto Ball Count", numberOfBalls);
        }
        for (StartingPosition position : StartingPosition.values()) {
            startingPositionChooser.addOption("Auto Starting Position", position);
        }

        if(option == DefaultChooserOptions.BALL_COUNT){
            return ballCountChooser;
        }
        else{
            return startingPositionChooser;
        }
    }

    //Sets path depending on the robot's starting position and the amount of balls interacted with in the autonomous path
    public static Command setPath(StartingPosition position, BallCount ballCount, SwerveDrive drive ){
        if(position == StartingPosition.TARMAC_ANY_SIDE){
            if(ballCount == BallCount.ZERO_BALL){
                return Paths.get0BallTaxi();
            }
            else if(ballCount == BallCount.ONE_BALL){
                return Paths.get1BallTaxi();
            }
            return Paths.get0BallTaxi();
        }
        else if(position == StartingPosition.EDGE_TARMAC_TWO_BALL_SIDE){
            if(ballCount == BallCount.TWO_BALL){
                return Paths.get2BallFarTarmac2BallSide();
            }
            else if(ballCount == BallCount.THREE_BALL){
                return Paths.get3BallFarTarmac2BallSide();
            }
            else if(ballCount == BallCount.FOUR_BALL){
                return Paths.get4BallFarTarmac2BallSide();
            }
            return Paths.get0BallTaxi();
        }
        else if(position == StartingPosition.MID_TARMAC_TWO_BALL_SIDE){
            if(ballCount == BallCount.TWO_BALL){
                return Paths.get2BallMidTarmac2BallSide();
            }
            else if(ballCount == BallCount.FOUR_BALL){
                return Paths.get4BallMidTarmac2BallSide();
            }
            return Paths.get0BallTaxi();
        }
        else if(position == StartingPosition.MID_TARMAC_ONE_BALL_SIDE){
            return Paths.get2BallMidTarmac1BallSide();
        }
        return new DefaultDriveCommandRobotOriented(drive);
    }

    public static Command getCommand(SwerveDrive drive) {
        return flywheelSubsystem != null ?
                new ParallelRaceGroup(
                    new WaitCommand(3),
                    new ZeroHoodMotorCommand(flywheelSubsystem)
                ).andThen(setPath(startingPositionChooser.getSelected(), ballCountChooser.getSelected(), drive))
                :
                setPath(startingPositionChooser.getSelected(), ballCountChooser.getSelected(), drive);
    }
}
