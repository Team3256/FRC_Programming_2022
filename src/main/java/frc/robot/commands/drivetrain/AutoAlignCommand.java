package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;
import static frc.robot.Constants.SwerveConstants.*;

import java.util.function.DoubleSupplier;

public class AutoAlignCommand extends CommandBase {
    private final SwerveDrive drivetrainSubsystem;

    private final DoubleSupplier joystickX;
    private final DoubleSupplier joystickY;
    private final DoubleSupplier limelightError;

    private PIDController thetaController;

    public AutoAlignCommand(SwerveDrive drivetrainSubsystem,
                                            DoubleSupplier joystickX,
                                            DoubleSupplier joystickY,
                                            DoubleSupplier limelightError) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.joystickX = joystickX;
        this.joystickY = joystickY;
        this.limelightError = limelightError;
        this.thetaController = new PIDController(SWERVE_TURRET_KP,SWERVE_TURRET_KD,SWERVE_TURRET_KD);

        addRequirements(drivetrainSubsystem);
    }

    public AutoAlignCommand(SwerveDrive drivetrainSubsystem) { // constructor that sets values to 0
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.joystickX = () -> 0;
        this.joystickY = () -> 0;
        this.limelightError = () -> 0;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        joystickX.getAsDouble(),
                        joystickY.getAsDouble(),
                        thetaController.calculate(drivetrainSubsystem.getGyroscopeRotation().getRadians(), limelightError.getAsDouble()),
                        drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}