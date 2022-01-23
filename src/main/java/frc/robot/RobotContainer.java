package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.DefaultDriveCommand;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
//import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeOn;
import frc.robot.helper.logging.RobotLogger;
//import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.ArrayList;
import java.util.List;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // The robot's subsystems and commands are defined here...
    private final SwerveDrive drivetrainSubsystem = new SwerveDrive();
    private final IntakeSubsystem intake = new IntakeSubsystem();
    private final Field2d field = new Field2d();
    private final XboxController controller = new XboxController(0);
    private static Trajectory currentTrajectory = new Trajectory();

    /**
     *
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
      RobotLogger.setup();
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotationx

       drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommand(
                drivetrainSubsystem,
                () -> -modifyAxis(controller.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(controller.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(controller.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
      Button rightBumper = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
        // Back button zeros the gyroscope
        new Button(controller::getAButton)
                // No requirements because we don't need to interrupt anything
                .whenPressed(drivetrainSubsystem::zeroGyroscope);
      
      rightBumper.whenHeld(new IntakeOn(intake));
    }

    public Command getAutonomousCommand() {
        return AutoChooser.getCommand();
    }

    public SendableChooser<Command> getCommandChooser() {
        return AutoChooser.getDefaultChooser(drivetrainSubsystem, intake);
    }

    public Trajectory getTrajectory() {
       return currentTrajectory;
    }

    public static void setCurrentTrajectory(Trajectory newTrajectory) {
        currentTrajectory = newTrajectory;
    }

    public void sendTrajectoryToDashboard() {
        field.getObject("traj").setTrajectory(getTrajectory());
    }

    public void autoOutputToDashboard() {
        field.setRobotPose(drivetrainSubsystem.getPose());
        SmartDashboard.putData("Field", field);
    }

    public void resetPose() {
        drivetrainSubsystem.resetOdometry(new Pose2d());
    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            if (value > 0.0) {
                return (value - deadband) / (1.0 - deadband);
            } else {
                return (value + deadband) / (1.0 - deadband);
            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        // Deadband
        value = deadband(value, 0.05);

        // Square the axis
        value = Math.copySign(value * value, value);

        return value;
    }
}