// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

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
import frc.robot.commands.shooter.ManualLeftTurret;
import frc.robot.commands.shooter.ManualRightTurret;
import frc.robot.commands.shooter.NinetyDegreeTurnTurret;
import frc.robot.subsystems.SwerveDrive;
import frc.robot.Constants.SwerveConstants;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.IntakeOn;
import frc.robot.commands.shooter.SetShooterFromTriggerDebug;
import frc.robot.helper.JoystickAnalogButton;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.IntakeSubsystem;
import java.util.ArrayList;
import java.util.List;
import frc.robot.subsystems.FlywheelSubsystem;
import frc.robot.subsystems.TurretSubsystem;

import java.util.Set;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

  private final TurretSubsystem turretSubsystem = new TurretSubsystem();
  private final XboxController controller = new XboxController(0);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    RobotLogger.setup();
      // Set up the default command for the drivetrain.
      // The controls are for field-oriented driving:
      // Left stick Y axis -> forward and backwards movement
      // Left stick X axis -> left and right movement
      // Right stick X axis -> rotationx

       
     configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    Button leftBumper = new JoystickButton(controller, XboxController.Button.kLeftBumper.value);
    leftBumper.whenHeld(new ManualLeftTurret(turretSubsystem));
    Button rightBumper = new JoystickButton(controller, XboxController.Button.kRightBumper.value);
    rightBumper.whenHeld(new ManualRightTurret(turretSubsystem));
    Button aButton = new JoystickButton(controller, XboxController.Button.kA.value);
    aButton.whenHeld(new NinetyDegreeTurnTurret(turretSubsystem));

    }
}