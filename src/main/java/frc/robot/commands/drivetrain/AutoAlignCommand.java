package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

public class AutoAlignCommand extends CommandBase {
    private final SwerveDrive drivetrainSubsystem;

    private final DoubleSupplier translationXSupplier;
    private final DoubleSupplier translationYSupplier;
    private final DoubleSupplier rotationSupplier;

    private PIDController thetaController;

    public AutoAlignCommand(SwerveDrive drivetrainSubsystem,
                                            DoubleSupplier translationXSupplier,
                                            DoubleSupplier translationYSupplier,
                                            DoubleSupplier rotationSupplier) {
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = translationXSupplier;
        this.translationYSupplier = translationYSupplier;
        this.rotationSupplier = rotationSupplier;
        this.thetaController = new PIDController(0,0,0);

        addRequirements(drivetrainSubsystem);
    }

    public AutoAlignCommand(SwerveDrive drivetrainSubsystem) { // constructor that sets values to 0
        this.drivetrainSubsystem = drivetrainSubsystem;
        this.translationXSupplier = () -> 0;
        this.translationYSupplier = () -> 0;
        this.rotationSupplier = () -> 0;

        addRequirements(drivetrainSubsystem);
    }

    @Override
    public void execute() {
        // You can use `new ChassisSpeeds(...)` for robot-oriented movement instead of field-oriented movement
        drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        translationXSupplier.getAsDouble(),
                        translationYSupplier.getAsDouble(),
                        thetaController.calculate(drivetrainSubsystem.getGyroscopeRotation().getRadians(), rotationSupplier.getAsDouble()),
                        drivetrainSubsystem.getGyroscopeRotation()
                )
        );
    }

    @Override
    public void end(boolean interrupted) {
        drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
    }
}