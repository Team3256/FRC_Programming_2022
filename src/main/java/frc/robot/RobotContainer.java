// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.BrownoutWatcher;
import frc.robot.commands.drivetrain.AutoAlignDriveContinuousCommand;
import frc.robot.commands.drivetrain.DefaultDriveCommandFieldOriented;
import frc.robot.helper.JoystickAnalogButton;
import frc.robot.hardware.Limelight;
import frc.robot.subsystems.SwerveDrive;

import java.awt.Robot;

import static frc.robot.Constants.SwerveConstants.AUTO_AIM_BREAKOUT_TOLERANCE;

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

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    private static Trajectory currentTrajectory = new Trajectory();

    /**
     *
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        CommandScheduler.getInstance().schedule(new BrownoutWatcher());

        Limelight.init();


        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {

        SmartDashboard.putData(CommandScheduler.getInstance());
        Button rightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        Button leftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);


        // Drivetrain Command
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotationx

        Command defaultDriveCommand = new DefaultDriveCommandFieldOriented(
                drivetrainSubsystem,
                () -> -modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -modifyAxis(driverController.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        );

        // Automatically Schedule Command when nothing else is scheduled
        drivetrainSubsystem.setDefaultCommand(defaultDriveCommand);



        // A button zeros the gyroscope
        new Button(driverController::getAButton)
                .whenPressed(drivetrainSubsystem::zeroGyroscope);

        // Left Bumper Enables Auto Align
        leftBumper.whenPressed(
                new AutoAlignDriveContinuousCommand(
                        drivetrainSubsystem,
                        () -> -modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -modifyAxis(operatorController.getRightX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND
                ), true);
        //Any Significant Movement in driver's X interrupt auto align
        new Button(()->Math.abs(driverController.getRightX()) > AUTO_AIM_BREAKOUT_TOLERANCE)
                .whenPressed(defaultDriveCommand);
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
        JoystickAnalogButton rightTrigger = new JoystickAnalogButton(driverController, XboxController.Axis.kRightTrigger.value);
        rightTrigger.setThreshold(0.01);


    }

    public void sendTrajectoryToDashboard() {
        field.getObject("traj").setTrajectory(getTrajectory());
    }

    public void robotOutputToDashboard() {
        SmartDashboard.putNumber("Modified Left Y", modifyAxis(driverController.getLeftY()));
        SmartDashboard.putNumber("Unmodified Left Y", (driverController.getLeftY()));
        SmartDashboard.putNumber("Modified Left X", modifyAxis(driverController.getLeftX()));
        SmartDashboard.putNumber("Unmodified Left X", (driverController.getLeftX()));
        SmartDashboard.putNumber("Modified Right X", modifyAxis(driverController.getRightX()));
        SmartDashboard.putNumber("Unmodified Right X", (driverController.getRightX()));
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
        double deadband = 0.1;
        value = deadband(value, deadband);

        if (value == 0) {
            return 0;
        }
        value = Math.copySign(Math.pow((((1 + deadband)*value) - deadband), 3), value);

        return value;
    }
}