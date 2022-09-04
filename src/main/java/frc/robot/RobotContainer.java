// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.*;
import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import io.github.oblarg.oblog.Logger;

import frc.robot.Constants.SwerveConstants;
import frc.robot.auto.AutoChooser;
import frc.robot.commands.WaitAndVibrateCommand;
import frc.robot.commands.drivetrain.AutoAlignDriveCommand;
import frc.robot.commands.drivetrain.DefaultDriveCommandFieldOriented;
import frc.robot.commands.drivetrain.DefaultDriveCommandRobotOriented;
import frc.robot.commands.hanger.*;
import frc.robot.commands.intake.IntakeOn;
import frc.robot.commands.intake.IntakeReverse;
import frc.robot.commands.shooter.SetShooterPIDFromInterpolation;
import frc.robot.commands.shooter.SetShooterPIDVelocityFromDashboard;
import frc.robot.commands.shooter.SetShooterPIDVelocityFromState;
import frc.robot.commands.shooter.ZeroHoodMotorCommand;
import frc.robot.commands.transfer.OuttakeFast;
import frc.robot.commands.transfer.TransferManualReverse;
import frc.robot.commands.transfer.TransferShootForward;
import frc.robot.hardware.Limelight;
import frc.robot.helper.ControllerUtil;
import frc.robot.helper.DPadButton;
import frc.robot.helper.JoystickAnalogButton;
import frc.robot.subsystems.*;

import java.awt.Robot;

import static frc.robot.Constants.ShooterConstants.ALL_SHOOTER_PRESETS;
import static frc.robot.Constants.SubsystemEnableFlags.*;
import static frc.robot.Constants.SwerveConstants.AUTO_AIM_BREAKOUT_TOLERANCE;
import static frc.robot.Constants.TransferConstants.MAX_BALL_COUNT;


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

    private ShooterSubsystem shooterSubsystem = null;
    private TransferSubsystem transferSubsystem = null;

    public HangerSubsystem hangerSubsystem = null;

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);
        Logger.configureLoggingAndConfig(this, false);

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
            SendableChooser<Command> commandChooser = getCommandChooser();
            if (commandChooser != null)
                SmartDashboard.putData(commandChooser);
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

    public void updateEntries() {
        Logger.updateEntries();
    }

    public Command getAutonomousCommand() {
        return AutoChooser.getCommand();
    }

    public SendableChooser<Command> getCommandChooser() {
        return AutoChooser.getDefaultChooser(drivetrainSubsystem, intakeSubsystem, shooterSubsystem, transferSubsystem);
    }

    private void initializeDrivetrain() {
        this.drivetrainSubsystem = new SwerveDrive();
    }

    private void initializeShooter() {
        this.shooterSubsystem = new ShooterSubsystem();
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
        JoystickAnalogButton driverRightTrigger = new JoystickAnalogButton(driverController, XboxController.Axis.kRightTrigger.value);
        driverRightTrigger.setThreshold(0.1);


        // Drivetrain Command
        // Set up the default command for the drivetrain.
        // The controls are for field-oriented driving:
        // Left stick Y axis -> forward and backwards movement
        // Left stick X axis -> left and right movement
        // Right stick X axis -> rotation X

        Command defaultDriveCommand = new DefaultDriveCommandFieldOriented(
                drivetrainSubsystem,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(driverController.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        );

        driverRightTrigger.toggleWhenActive(new DefaultDriveCommandRobotOriented(
                drivetrainSubsystem,
                () -> ControllerUtil.modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> ControllerUtil.modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(driverController.getRightX()) * SwerveConstants.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND
        ));

        // Automatically Schedule Command when nothing else is scheduled
        drivetrainSubsystem.setDefaultCommand(defaultDriveCommand);

        // A button zeros the gyroscope
        driverAButton.whenPressed(drivetrainSubsystem::zeroGyroscope);

        Command autoAlign = new AutoAlignDriveCommand(
                drivetrainSubsystem,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                () -> -ControllerUtil.modifyAxis(operatorController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                transferSubsystem::isShooting
        );

        if (LIMELIGHT) {
            // Left Bumper Enables Auto Align
            driverLeftBumper.whenPressed(
                autoAlign
            );
        }

        // Any Significant Movement in driver's X interrupt auto align
        new Button(()->Math.abs(driverController.getRightX()) > AUTO_AIM_BREAKOUT_TOLERANCE)
                .cancelWhenActive(autoAlign);
    }

    private void configureShooter() {
        DPadButton dPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
        Button driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        JoystickAnalogButton operatorLeftTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kLeftTrigger.value);
        operatorLeftTrigger.setThreshold(0.1);

        driverLeftBumper.whenHeld(new SetShooterPIDVelocityFromDashboard(shooterSubsystem));
        dPadUp.whenHeld(new ZeroHoodMotorCommand(shooterSubsystem));

        // Vibrations
        if (TRANSFER) {
            new Button(() -> transferSubsystem.getCurrentBallCount() >= MAX_BALL_COUNT).whenPressed(new WaitAndVibrateCommand(driverController, 0.1, 0.1));
        }

        if(LIMELIGHT) {
             driverLeftBumper.whileActiveOnce(
                     new SetShooterPIDFromInterpolation(shooterSubsystem, transferSubsystem::isShooting, driverController)
             );

             operatorLeftTrigger.whileActiveOnce(
                     new SetShooterPIDFromInterpolation(shooterSubsystem, transferSubsystem::isShooting, driverController)
             );
        }

    }

    private void configureTransfer() {
        JoystickAnalogButton driverLeftTrigger = new JoystickAnalogButton(driverController, XboxController.Axis.kLeftTrigger.value);

      driverLeftTrigger.whenHeld(new TransferShootForward(transferSubsystem, shooterSubsystem), false);
//        operatorLeftTrigger.whenHeld(new TransferIndexForward(transferSubsystem), false);
    }

    private void configureIntake() {
        JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        JoystickButton driverBButton = new JoystickButton(driverController, XboxController.Button.kB.value);
        JoystickButton driverYButton = new JoystickButton(driverController, XboxController.Button.kY.value);
        JoystickAnalogButton operatorRightTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kRightTrigger.value);
        operatorRightTrigger.setThreshold(0.1);

        // Operator's Intake Up Button
        operatorRightTrigger.whenHeld(new IntakeOn(intakeSubsystem));
        driverRightBumper.whenHeld(new IntakeOn(intakeSubsystem));

        if (TRANSFER) {
            driverBButton.whenHeld(
                    new ParallelCommandGroup(
                            new IntakeReverse(intakeSubsystem),
                            new TransferManualReverse(transferSubsystem)
                    ), false
            );

            driverYButton.whenHeld(new OuttakeFast(transferSubsystem, intakeSubsystem));
        }
    }

    private void configureHanger() {
        JoystickButton operatorAButton = new JoystickButton(operatorController, XboxController.Button.kA.value);
        JoystickButton operatorXButton = new JoystickButton(operatorController, XboxController.Button.kX.value);
        JoystickButton operatorYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
        JoystickButton operatorBButton = new JoystickButton(operatorController, XboxController.Button.kB.value);

        JoystickButton operatorStartButton = new JoystickButton(operatorController, XboxController.Button.kStart.value);
        JoystickButton operatorRB = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);

        DPadButton operatorDPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);

        operatorXButton.whenHeld(new HangerZeroRetract(hangerSubsystem), false);
        operatorAButton.whenHeld(
                (new HangerSyncOnBar(hangerSubsystem))
                .andThen(new HangerRetractForHang(hangerSubsystem))
                , false);
        operatorYButton.whenHeld(new HangerExtend(hangerSubsystem), false);

        operatorBButton.whenHeld(new HangerPartial(hangerSubsystem), false);

        operatorStartButton.whenHeld(new SequentialCommandGroup(
                new HangerPartial(hangerSubsystem),
                new InstantCommand(()->{System.out.println("Finsihed Partial!");}),
                new HangerPneumaticSlant(hangerSubsystem, intakeSubsystem),
                new InstantCommand(()->{System.out.println("Finsihed Slant!");}),
                new WaitCommand(0.5),
                new HangerExtend(hangerSubsystem),
                new HangerPneumaticUpright(hangerSubsystem)
        ), false);

        operatorRB
                .whenHeld(new HangerPneumaticSlant(hangerSubsystem, intakeSubsystem))
                .whenReleased(new HangerPneumaticUpright(hangerSubsystem));

        operatorDPadDown.whenHeld(new HangerRetractForHang(hangerSubsystem));

        // Auto Retract
        new Trigger(()->((hangerSubsystem.getLeftPosition() > 10000 || hangerSubsystem.getRightPosition() > 10000)))
                .whenActive(new HangerZeroRetract(hangerSubsystem));
    }

    public void zeroSubsystems() {
        if (SHOOTER)
            CommandScheduler.getInstance().schedule(new ZeroHoodMotorCommand(shooterSubsystem));
        if (HANGER)
            CommandScheduler.getInstance().schedule(new HangerZeroRetract(hangerSubsystem));
    }

    public void resetPose() {
        drivetrainSubsystem.resetOdometry(new Pose2d());
    }
}
