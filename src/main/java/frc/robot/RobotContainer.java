// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.drivetrain.AutoAlignDriveContinuousCommand;
import frc.robot.commands.drivetrain.DefaultDriveCommandFieldOriented;
import frc.robot.commands.hanger.*;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.intake.IntakeReverse;
import frc.robot.commands.shooter.*;
import frc.robot.commands.transfer.TransferIndexForward;
import frc.robot.commands.transfer.TransferManualReverse;
import frc.robot.hardware.Limelight;
import frc.robot.helper.ControllerUtil;
import frc.robot.helper.DPadButton;
import frc.robot.helper.JoystickAnalogButton;
import frc.robot.subsystems.*;

import java.awt.Robot;

import static frc.robot.Constants.SubsystemEnableFlags.*;
import static frc.robot.Constants.SwerveConstants.AUTO_AIM_BREAKOUT_TOLERANCE;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {

    // The robot's subsystems and commands are defined here...
    public SwerveDrive drivetrainSubsystem = null;
    private IntakeSubsystem intakeSubsystem = null;

    private FlywheelSubsystem flywheelSubsystem = null;
    private TransferSubsystem transferSubsystem = null;

    private HangerSubsystem hangerSubsystem = null;

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
       // CommandScheduler.getInstance().schedule(new PDHFaultWatcher());


        // Initialize Active Subsystems
        if (LIMELIGHT)
            Limelight.init();
        if (DRIVETRAIN)
            initializeDrivetrain();
        if (SHOOTER)
            initializeShooter();
        if (TRANSFER)
            initializeTransfer();
        if (INTAKE)
            initializeIntake();
        if (HANGER)
            initializeHanger();

        // Configure Enabled Subsystems
        if (DRIVETRAIN) {
            configureDrivetrain();
            if (getCommandChooser() != null)
                SmartDashboard.putData(getCommandChooser());
        }
        if (SHOOTER)
            configureDebugShooter();
        if (TRANSFER)
            configureTransfer();
        if (INTAKE)
            configureIntake();
        if (HANGER)
            configureHanger();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */

    public Command getAutonomousCommand() {
        if (DRIVETRAIN && INTAKE && SHOOTER && TRANSFER)
            return AutoChooser.getCommand();
        else
            return null;
    }

    public SendableChooser<Command> getCommandChooser() {
        if (DRIVETRAIN && INTAKE && SHOOTER && TRANSFER)
            return AutoChooser.getDefaultChooser(drivetrainSubsystem, intakeSubsystem, flywheelSubsystem, transferSubsystem);
        else
            return null;
    }

    private void initializeDrivetrain() {
        this.drivetrainSubsystem = new SwerveDrive();
    }

    private void initializeShooter() {
        this.flywheelSubsystem = new FlywheelSubsystem();
    }

    private void initializeTransfer() {
        this.transferSubsystem = new TransferSubsystem();
    }

    private void initializeIntake() {
        this.intakeSubsystem = new IntakeSubsystem();
    }

    private void initializeHanger() {
        this.hangerSubsystem = new HangerSubsystem();
    }

    private void configureDrivetrain() {
        Button driverAButton = new JoystickButton(driverController, XboxController.Button.kA.value);
        Button leftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        Button rightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);

        // Drivetrain Command
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotationx

        Command defaultDriveCommand = new DefaultDriveCommandFieldOriented(
                drivetrainSubsystem,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(driverController.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        );

        // Automatically Schedule Command when nothing else is scheduled
        drivetrainSubsystem.setDefaultCommand(defaultDriveCommand);

        // A button zeros the gyroscope
        driverAButton.whenPressed(drivetrainSubsystem::zeroGyroscope);

        // Left Bumper Enables Auto Align
        leftBumper.whenPressed(
                new AutoAlignDriveContinuousCommand(
                        drivetrainSubsystem,
                        () -> -ControllerUtil.modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -ControllerUtil.modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -ControllerUtil.modifyAxis(operatorController.getRightX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND
                ), true);

        //Any Significant Movement in driver's X interrupt auto align
        new Button(()->Math.abs(driverController.getRightX()) > AUTO_AIM_BREAKOUT_TOLERANCE)
                .whenPressed(defaultDriveCommand);
    }

    private void configureShooter() {
        JoystickAnalogButton rightTrigger = new JoystickAnalogButton(driverController, XboxController.Axis.kRightTrigger.value);
        rightTrigger.setThreshold(0.01);
//
//        DPadButton dPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
//        DPadButton dPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);
//
//        if (LIMELIGHT)
//            rightTrigger.whenHeld(new AutoAimShooter(flywheelSubsystem));
//
//        dPadUp.whenPressed(new IncreasePresetForShooter(flywheelSubsystem));
//        dPadDown.whenPressed(new DecreasePresetForShooter(flywheelSubsystem));

        rightTrigger.whenPressed(new SetShooterFromCustomDashboardConfig(flywheelSubsystem));
    }

    private void configureDebugShooter(){
        JoystickAnalogButton rightTrigger = new JoystickAnalogButton(driverController, XboxController.Axis.kRightTrigger.value);
        JoystickAnalogButton aButton = new JoystickAnalogButton(driverController, XboxController.Button.kA.value);

        rightTrigger.setThreshold(0.01);

        rightTrigger.whenHeld(new SetShooterFromCustomDashboardConfig(flywheelSubsystem));
        aButton.whenPressed(new ZeroHoodMotorCommand(flywheelSubsystem));


    }

    private void configureTransfer() {
        DPadButton dPadButtonUp = new DPadButton(driverController, DPadButton.Direction.UP);
        DPadButton dPadButtonDown = new DPadButton(driverController, DPadButton.Direction.DOWN);

        dPadButtonDown.whenHeld(new TransferManualReverse(transferSubsystem), false);
        dPadButtonUp.whenHeld(new TransferIndexForward(transferSubsystem), false);
    }

    private void configureIntake() {
        JoystickButton rightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        JoystickButton bButton = new JoystickButton(driverController, XboxController.Button.kB.value);

        rightBumper.whenHeld(new IntakeOn(intakeSubsystem));
        bButton.whenHeld(new IntakeReverse(intakeSubsystem));
    }

    private void configureHanger() {
        //TODO: IF we are doing traversal, Ensure that Intake is Down with Commands

        JoystickButton xButton = new JoystickButton(driverController, XboxController.Button.kX.value);
        JoystickButton kY =  new JoystickButton(driverController, XboxController.Button.kY.value);

        DPadButton dLeft = new DPadButton(driverController, DPadButton.Direction.LEFT);
        DPadButton dRight = new DPadButton(driverController, DPadButton.Direction.RIGHT);
        DPadButton dUp = new DPadButton(driverController, DPadButton.Direction.UP);
        DPadButton dDown = new DPadButton(driverController, DPadButton.Direction.DOWN);


        xButton.whenHeld(new HangerZeroRetract(hangerSubsystem));

        dUp.whenHeld(new HangerExtend(hangerSubsystem));
        dDown.whenActive(new HangerRetractForHang(hangerSubsystem));
        dLeft.whenActive(new HangerPneumaticSlant(hangerSubsystem));
        dRight.whenActive(new HangerPneumaticUpright(hangerSubsystem));
    }

    public void resetPose() {
        drivetrainSubsystem.resetOdometry(new Pose2d());
    }

    private static double deadband(double value, double deadband) {
        return (Math.abs(value) > deadband) ? value : 0.0;
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