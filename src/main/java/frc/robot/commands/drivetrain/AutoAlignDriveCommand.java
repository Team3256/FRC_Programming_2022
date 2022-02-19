package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveConstants.*;

public class AutoAlignDriveCommand extends PIDCommand {

    private final SwerveDrive drivetrainSubsystem;
    private final DoubleSupplier joystickX;
    private final DoubleSupplier joystickY;
    private final DoubleSupplier limeLightThetaError;

    public AutoAlignDriveCommand(SwerveDrive drivetrainSubsystem,
                                 DoubleSupplier joystickX,
                                 DoubleSupplier joystickY,
                                 DoubleSupplier limeLightThetaError) {
        super(new PIDController(SWERVE_TURRET_KP, SWERVE_TURRET_KI, SWERVE_TURRET_KD),
                Limelight::getTx,
                0,
                output -> drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                        joystickX.getAsDouble(),
                        joystickY.getAsDouble(),
                        output,
                        drivetrainSubsystem.getGyroscopeRotation()
                )),
                drivetrainSubsystem);
        getController().enableContinuousInput(-180,180);
        getController().setTolerance(TURN_TOLERANCE, TURN_RATE_TOLERANCE);

        this.drivetrainSubsystem = drivetrainSubsystem;
        this.joystickX = joystickX;
        this.joystickY = joystickY;
        this.limeLightThetaError = limeLightThetaError;
    }


    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}