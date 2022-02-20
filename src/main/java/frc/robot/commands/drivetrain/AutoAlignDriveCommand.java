package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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
                output -> {
                    //Use 0 min for moving, use 0.4 for non-moving
            double speed = Math.sqrt(Math.pow(joystickX.getAsDouble(),2) + Math.pow(joystickY.getAsDouble(),2));
            drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                        joystickX.getAsDouble(),
                        joystickY.getAsDouble(),
                         speed > 0.1 ? output : output + Math.copySign(0.4, output) ,
                        drivetrainSubsystem.getGyroscopeRotation()
                ));

                    SmartDashboard.putNumber("PID OUT",output < 0 ? output - 0.4 : output +0.4 );
                },
                drivetrainSubsystem);
        getController().enableContinuousInput(-180,180);

        this.drivetrainSubsystem = drivetrainSubsystem;
        this.joystickX = joystickX;
        this.joystickY = joystickY;
        this.limeLightThetaError = limeLightThetaError;
        System.out.println("INIT PID");
    }


    @Override
    public boolean isFinished() {
        return false;
    }

}