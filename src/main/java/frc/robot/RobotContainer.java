// Copyright (c) WarriorBorgs.
// Open Source Software; you can modify and/or share it under the terms of
// the WarriorBorgs BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

import edu.wpi.first.wpilibj2.command.*;

import edu.wpi.first.wpilibj2.command.button.Button;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
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
import frc.robot.commands.shooter.SetShooterWhileMoving;
import frc.robot.commands.shooter.ZeroHoodMotorCommand;
import frc.robot.commands.transfer.OuttakeFast;
import frc.robot.commands.transfer.TransferManualReverse;
import frc.robot.commands.transfer.TransferShootForward;
import frc.robot.hardware.Limelight;
import frc.robot.helper.ControllerUtil;
import frc.robot.helper.DPadButton;
import frc.robot.helper.JoystickAnalogButton;
import frc.robot.subsystems.*;
import io.github.oblarg.oblog.annotations.Log;

import java.awt.Robot;

import static frc.robot.Constants.SubsystemEnableFlags.*;
import static frc.robot.Constants.GOING_CRAZY;
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
    @Log
    public SwerveDrive drivetrainSubsystem = null;
    @Log
    private IntakeSubsystem intakeSubsystem = null;
    @Log
    private ShooterSubsystem shooterSubsystem = null;
    @Log
    private TransferSubsystem transferSubsystem = null;
    @Log
    public HangerSubsystem hangerSubsystem = null;

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);
    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        LiveWindow.disableAllTelemetry();
        LiveWindow.setEnabled(false);

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
            if (!GOING_CRAZY) {
                driverLeftBumper.whenPressed(
                    autoAlign
                );
            }

            if (SHOOTER && GOING_CRAZY) {
                driverLeftBumper.whenPressed(
                    new SetShooterWhileMoving(drivetrainSubsystem, shooterSubsystem, 
                        () -> -ControllerUtil.modifyAxis(driverController.getLeftY()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND,
                        () -> -ControllerUtil.modifyAxis(driverController.getLeftX()) * SwerveConstants.MAX_VELOCITY_METERS_PER_SECOND
                    )
                );
                
            }
        }

        // Any Significant Movement in driver's X interrupt auto align
        new Button(()->Math.abs(driverController.getRightX()) > AUTO_AIM_BREAKOUT_TOLERANCE)
                .cancelWhenActive(autoAlign);
    }

    private void configureShooter() {
        JoystickButton operatorXButton = new JoystickButton(operatorController, XboxController.Button.kX.value);

        Button driverLeftBumper = new JoystickButton(driverController, XboxController.Button.kLeftBumper.value);
        JoystickAnalogButton operatorLeftTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kLeftTrigger.value);
        operatorLeftTrigger.setThreshold(0.1);

        // driverLeftBumper.whenHeld(new SetShooterPIDVelocityFromDashboard(shooterSubsystem));
        driverLeftBumper.whenHeld(new SetShooterPIDVelocityFromState(shooterSubsystem, () -> new ShooterState(1200, 0)));

        operatorXButton.whenHeld(new ZeroHoodMotorCommand(shooterSubsystem));


        // Vibrations
        if (TRANSFER) {
            new Button(() -> transferSubsystem.getCurrentBallCount() >= MAX_BALL_COUNT).whenPressed(new WaitAndVibrateCommand(driverController, 0.1, 0.1));
        }

        if(LIMELIGHT) { 
                if (!GOING_CRAZY) { // shooting while moving is in configureDrivetrain cause it uses drive
                    // driverLeftBumper.whileActiveOnce(
                    //      new SetShooterPIDFromInterpolation(shooterSubsystem, drivetrainSubsystem::getEstimatedDistance, Limelight::isTargetDetected)
                    // );
                    //
                    // operatorLeftTrigger.whileActiveOnce(
                    //      new SetShooterPIDFromInterpolation(shooterSubsystem, drivetrainSubsystem::getEstimatedDistance, Limelight::isTargetDetected)
                    // );
                }
            }
        }

    private void configureTransfer() {
        JoystickAnalogButton driverLeftTrigger = new JoystickAnalogButton(driverController, XboxController.Axis.kLeftTrigger.value);
        JoystickAnalogButton operatorRightTrigger = new JoystickAnalogButton(operatorController, XboxController.Axis.kRightTrigger.value);
        operatorRightTrigger.setThreshold(0.1);

        driverLeftTrigger.whenHeld(new TransferShootForward(transferSubsystem, shooterSubsystem), false);
        operatorRightTrigger.whenHeld(new TransferShootForward(transferSubsystem, shooterSubsystem), false);
    }

    private void configureIntake() {
        JoystickButton driverRightBumper = new JoystickButton(driverController, XboxController.Button.kRightBumper.value);
        JoystickButton driverBButton = new JoystickButton(driverController, XboxController.Button.kB.value);
        JoystickButton driverYButton = new JoystickButton(driverController, XboxController.Button.kY.value);

        // Operator's Intake Up Button
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
        JoystickButton operatorYButton = new JoystickButton(operatorController, XboxController.Button.kY.value);
        JoystickButton operatorBButton = new JoystickButton(operatorController, XboxController.Button.kB.value);

        JoystickButton operatorStartButton = new JoystickButton(operatorController, XboxController.Button.kStart.value);
        JoystickButton operatorRB = new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);

        DPadButton operatorDPadDown = new DPadButton(operatorController, DPadButton.Direction.DOWN);
        DPadButton operatorDPadUp = new DPadButton(operatorController, DPadButton.Direction.UP);
        DPadButton operatorDPadLeft = new DPadButton(operatorController, DPadButton.Direction.LEFT);
        DPadButton operatorDPadRight = new DPadButton(operatorController, DPadButton.Direction.RIGHT);

        operatorAButton.whenHeld(new HangerZeroRetract(hangerSubsystem), false);
        operatorBButton.whenHeld(
                (new HangerSyncOnBar(hangerSubsystem))
                .andThen(new HangerRetractForHang(hangerSubsystem))
                , false);

        operatorYButton.whenHeld(new HangerExtend(hangerSubsystem), false);

        operatorDPadUp.whenHeld(new HangerPartial(hangerSubsystem), false);

        operatorStartButton.whenHeld(new SequentialCommandGroup(
                new HangerPartial(hangerSubsystem),
                new InstantCommand(()->{System.out.println("Finsihed Partial!");}),
                new HangerPneumaticSlant(hangerSubsystem, intakeSubsystem),
                new InstantCommand(()->{System.out.println("Finsihed Slant!");}),
                new WaitCommand(0.5),
                new HangerExtend(hangerSubsystem),
                new HangerPneumaticUpright(hangerSubsystem)
        ), false);

        operatorDPadRight.whenHeld(new HangerPneumaticSlant(hangerSubsystem, intakeSubsystem));
        operatorDPadLeft.whenHeld(new HangerPneumaticUpright(hangerSubsystem));

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
