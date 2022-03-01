// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.BrownoutWatcher;
import frc.robot.commands.drivetrain.AutoAlignDriveContinuousCommand;
import frc.robot.commands.drivetrain.DefaultDriveCommandFieldOriented;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.hardware.Limelight;
import frc.robot.helper.BallColor;
import frc.robot.helper.CANdle.CANdleSystem;
import frc.robot.helper.JoystickAnalogButton;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.SwerveDrive;

import java.awt.Robot;

import static frc.robot.Constants.CANdleConstants.BALL_PATTERN;
import static frc.robot.Constants.SwerveConstants.AUTO_AIM_BREAKOUT_TOLERANCE;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    CANdleSystem caNdleSystem = new CANdleSystem(null);

    /**
     *
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {

        caNdleSystem.init();


        new Button(new XboxController(0)::getAButton).whenPressed(new InstantCommand(()->BALL_PATTERN.update(BallColor.BLUE, BallColor.RED)));

    }

    private static double deadband(double value, double deadband) {
        if (Math.abs(value) > deadband) {
            return value;
//            if (value > 0.0) {
//                return (value - deadband) / (1.0 - deadband);
//            } else {
//                return (value + deadband) / (1.0 - deadband);
//            }
        } else {
            return 0.0;
        }
    }

    private static double modifyAxis(double value) {
        double deadband = 0.05;
        value = deadband(value, deadband);

        if (value == 0) {
            return 0;
        }

        SmartDashboard.setDefaultNumber("Joystick Input Exponential Power", 3);
//
        double exp = SmartDashboard.getNumber("Joystick Input Exponential Power", 3);
        value = Math.copySign(Math.pow((((1 + deadband)*value) - deadband), exp), value);

        return value;
    }
}