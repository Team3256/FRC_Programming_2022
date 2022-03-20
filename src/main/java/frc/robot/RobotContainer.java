// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.commands.drivetrain.AutoAlignDriveContinuousCommand;
import frc.robot.commands.drivetrain.DefaultDriveCommandFieldOriented;
import frc.robot.commands.drivetrain.DefaultDriveCommandRobotOriented;
import frc.robot.commands.hanger.*;
import frc.robot.commands.intake.IntakeOff;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.intake.IntakeReverse;
import frc.robot.commands.shooter.*;
import frc.robot.commands.transfer.TransferIndexForward;
import frc.robot.commands.transfer.TransferManualReverse;
import frc.robot.commands.transfer.TransferShootForward;
import frc.robot.hardware.Limelight;
import frc.robot.helper.ControllerUtil;
import frc.robot.helper.DPadButton;
import frc.robot.helper.JoystickAnalogButton;
import frc.robot.subsystems.*;

import java.awt.Robot;

import static frc.robot.Constants.SubsystemEnableFlags.*;
import static frc.robot.Constants.SwerveConstants.AUTO_AIM_BREAKOUT_TOLERANCE;
import static frc.robot.Constants.TransferConstants.*;


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
            configureShooter();
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
        Button driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        JoystickAnalogButton driverLeftTrigger = new JoystickAnalogButton(driverController, XboxController.Axis.kLeftTrigger.value);


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

        driverLeftTrigger.toggleWhenActive(new DefaultDriveCommandRobotOriented(
                drivetrainSubsystem,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(driverController.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        // Automatically Schedule Command when nothing else is scheduled
        drivetrainSubsystem.setDefaultCommand(defaultDriveCommand);

        // A button zeros the gyroscope
        driverAButton.whenPressed(drivetrainSubsystem::zeroGyroscope);

        Command autoAlign = new AutoAlignDriveContinuousCommand(
                drivetrainSubsystem,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(operatorController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND
        );

        if(LIMELIGHT) {
            // Left Bumper Enables Auto Align
            driverLeftBumper.whenPressed(
                autoAlign
            );
        }

        //Any Significant Movement in driver's X interrupt auto align
        new Button(()->Math.abs(driverController.getRightX()) > AUTO_AIM_BREAKOUT_TOLERANCE)
                .cancelWhenActive(autoAlign);
    }

    private void configureShooter() {
        DPadButton dPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
        DPadButton dPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);
        DPadButton dPadRight = new DPadButton(operatorController, DPadButton.Direction.RIGHT);
        DPadButton dPadLeft= new DPadButton(operatorController, DPadButton.Direction.LEFT);

        JoystickAnalogButton operatorRightTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kRightTrigger.value);
        JoystickAnalogButton operatorLeftTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kLeftTrigger.value);


//        dPadUp.whenPressed(new SetShooterPreset(flywheelSubsystem, ShooterLocationPreset.FENDER));
//        dPadDown.whenPressed(new SetShooterPreset(flywheelSubsystem, ShooterLocationPreset.TARMAC_MIDDLE_VERTEX));
//        dPadRight.whenPressed(new SetShooterPreset(flywheelSubsystem, ShooterLocationPreset.TARMAC_SIDE_VERTEX));
//        dPadLeft.whenPressed(new SetShooterPreset(flywheelSubsystem, ShooterLocationPreset.TRUSS));

        operatorRightTrigger.whenHeld( new SetShooterPIDVelocityFromDashboard(flywheelSubsystem, ()->0));
        if (TRANSFER) {
            operatorLeftTrigger.whenHeld(new TransferShootForward(transferSubsystem), false);
            new Button(()-> transferSubsystem.getCurrentBallCount() >= MAX_BALL_COUNT).whenPressed(new WaitAndVibrateCommand(driverController, 0.5));
        }
        dPadUp.whenHeld(new ZeroHoodMotorCommand(flywheelSubsystem)).whenPressed(new InstantCommand(()->System.out.println("Activated Zero")));


    }

    private void configureDebugShooter(){
        JoystickAnalogButton rightTrigger = new JoystickAnalogButton(driverController, XboxController.Axis.kRightTrigger.value);
        JoystickAnalogButton leftTrigger  = new JoystickAnalogButton(driverController, XboxController.Axis.kLeftTrigger.value);
        JoystickButton aButton = new JoystickButton(driverController, XboxController.Button.kA.value);

        JoystickAnalogButton operatorLeftTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kLeftTrigger.value);
        operatorLeftTrigger.setThreshold(0.1);

//        DPadButton dPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
//        DPadButton dPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);
//        DPadButton dPadRight = new DPadButton(operatorController, DPadButton.Direction.RIGHT);
//        DPadButton dPadLeft= new DPadButton(operatorController, DPadButton.Direction.LEFT);

        rightTrigger.setThreshold(0.01);

        rightTrigger.whenHeld(new SetShooterFromCustomDashboardConfig(flywheelSubsystem));
        leftTrigger.whenHeld(new SetShooterFromLocationPreset(flywheelSubsystem));
        aButton.whenPressed(new ZeroHoodMotorCommand(flywheelSubsystem)).whenPressed(new InstantCommand(()->System.out.println("Activated Zero")));

        operatorLeftTrigger.whenHeld(new TransferIndexForward(transferSubsystem), false);
//        dPadUp.whenPressed(new SetShooterPreset(flywheelSubsystem, ShooterLocationPreset.FENDER));
//        dPadDown.whenPressed(new SetShooterPreset(flywheelSubsystem, ShooterLocationPreset.TARMAC_MIDDLE_VERTEX));
//        dPadRight.whenPressed(new SetShooterPreset(flywheelSubsystem, ShooterLocationPreset.TARMAC_SIDE_VERTEX));
//        dPadLeft.whenPressed(new SetShooterPreset(flywheelSubsystem, ShooterLocationPreset.TRUSS));
    }

    private void configureTransfer() {

    }

    private void configureIntake() {
        JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        JoystickButton operatorBButton = new JoystickButton(operatorController, XboxController.Button.kB.value);
        JoystickButton operatorRightBumper = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);

        // Operator's Intake Up Button
        operatorRightBumper.whenPressed(new IntakeOff(intakeSubsystem));


        driverRightBumper.toggleWhenActive(new IntakeOn(intakeSubsystem)); // TODO: bad

        if (TRANSFER)
            operatorBButton.whenHeld(
                    new ParallelCommandGroup(
                            new IntakeReverse(intakeSubsystem),
                            new TransferManualReverse(transferSubsystem)
                    )
            );



    }

    private void configureHanger() {
        //TODO: IF we are doing traversal, Ensure that Intake is Down with Commands

        JoystickButton operatorAButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
        JoystickButton operatorXButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
        JoystickButton operatorYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);

        DPadButton operatorLeftDpad = new DPadButton(operatorController, DPadButton.Direction.LEFT);
        DPadButton operatorRightDpad = new DPadButton(operatorController, DPadButton.Direction.RIGHT);

        operatorXButton.whenHeld(new HangerZeroRetract(hangerSubsystem));

        operatorLeftDpad.toggleWhenActive(new HangerTogglePneumatics(hangerSubsystem, intakeSubsystem));
        operatorRightDpad.whenHeld(new HangerPartial(hangerSubsystem));

        operatorAButton.whenHeld(new HangerRetractForHang(hangerSubsystem));
        operatorYButton.whenHeld(new HangerExtend(hangerSubsystem));
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
