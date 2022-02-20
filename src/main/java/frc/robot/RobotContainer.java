// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.BrownoutWatcher;
import frc.robot.commands.drivetrain.AutoAlignDriveCommand;
import frc.robot.commands.drivetrain.DefaultDriveCommandRobotOriented;
import frc.robot.commands.drivetrain.DefaultDriveCommandFieldOriented;
import frc.robot.commands.hanger.AutoHang;
import frc.robot.subsystems.HangerSubsystem;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.shooter.SetShooterFromTriggerDebug;
import frc.robot.helper.JoystickAnalogButton;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.IntakeSubsystem;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import frc.robot.subsystems.FlywheelSubsystem;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    private final SwerveDrive drivetrainSubsystem = new SwerveDrive();

    private final Field2d field = new Field2d();

    private final XboxController controller = new XboxController(0);
    private static Trajectory currentTrajectory = new Trajectory();

    /**
     *
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        CommandScheduler.getInstance().schedule(new BrownoutWatcher());

        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotationx

       drivetrainSubsystem.setDefaultCommand(new DefaultDriveCommandFieldOriented(
                drivetrainSubsystem,
                () -> -modifyAxis(controller.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(controller.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(controller.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

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

        Limelight.init();
        // Back button zeros the gyroscope
        new Button(controller::getAButton)
                .whenPressed(drivetrainSubsystem::zeroGyroscope);
        rightBumper.whenHeld(new AutoAlignDriveCommand(
                        drivetrainSubsystem,
                        () -> -modifyAxis(controller.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -modifyAxis(controller.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -modifyAxis(controller.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
                ));
    }

    public void logStuff(){
        SmartDashboard.putNumber("Controller Rot", -modifyAxis(controller.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND);
    }


    public Command getAutonomousCommand() {
        return AutoChooser.getCommand();
    }

    public SendableChooser<Command> getCommandChooser() {
        return null;
    }


    public Trajectory getTrajectory() {
       return currentTrajectory;
    }

    public static void setCurrentTrajectory(Trajectory newTrajectory) {
        currentTrajectory = newTrajectory;
    }

    private void configureShooter() {
        JoystickAnalogButton rightTrigger = new JoystickAnalogButton(controller, XboxController.Axis.kRightTrigger.value);
        rightTrigger.setThreshold(0.01);


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

        double deadband = 0.05;
        value = deadband(value, deadband);

        SmartDashboard.setDefaultNumber("exponential value", 3);

        double exp = SmartDashboard.getNumber("exponential value", 3);
        value = Math.copySign(Math.pow(value, exp), value);

        return value;
    }
}