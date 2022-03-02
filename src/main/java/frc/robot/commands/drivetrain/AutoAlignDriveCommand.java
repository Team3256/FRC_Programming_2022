package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.CANdleConstants.AUTO_AIM_PATTERN;
import static frc.robot.Constants.SwerveConstants.*;

public class AutoAlignDriveCommand extends PIDCommand {


    @Override
    public void initialize() {
        Limelight.enable();
        AUTO_AIM_PATTERN.update(true);
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.disable();
        AUTO_AIM_PATTERN.update(false);
    }

    /**
     * Rotates SwerveDrive toward target until at tolerance.
     * Meant for Auto, Not meant for tuning / teleop.
     *
     * @param drivetrainSubsystem drivetrain instance
     * @param joystickX Driver's Translation X
     * @param joystickY Driver's Translation Y
     */
    public AutoAlignDriveCommand (SwerveDrive drivetrainSubsystem,
                                            DoubleSupplier joystickX,
                                            DoubleSupplier joystickY) {

        super(new PIDController(SWERVE_TURRET_KP, SWERVE_TURRET_KI, SWERVE_TURRET_KD),
                Limelight::getTx, //Measurement Source
                0, //Set point
                pidOutput -> { //Using output

                    //Save some Computation from Sqrt
                    double speedSquared = Math.pow(joystickX.getAsDouble(),2) + Math.pow(joystickY.getAsDouble(),2);

                    // Use translation from Driver, Rotation is from PID
                    // Ternary Explanation: Since while moving we can easily rotate, we don't mess with the PID
                    // but while NOT moving, the motors need more power in order to actually move, so
                    // we add a Constant, (Using copysign to either add or subtract depending on sign)
                    double motorOutput = speedSquared > Math.pow(0.1,2) ?
                            pidOutput :
                            pidOutput + Math.copySign(SWERVE_TURRET_STATIONARY_MIN, pidOutput);

                    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                            joystickX.getAsDouble(),
                            joystickY.getAsDouble(),
                            motorOutput,
                            drivetrainSubsystem.getGyroscopeRotation()
                    ));
                },
                drivetrainSubsystem);
        getController().enableContinuousInput(-180,180);
        getController().setTolerance(TURN_TOLERANCE, TURN_RATE_TOLERANCE);
        SmartDashboard.putData("Auto AIM PID", getController());

    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint();
    }

}