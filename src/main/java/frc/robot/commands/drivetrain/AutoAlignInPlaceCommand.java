package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.SwerveDrive;
import edu.wpi.first.wpilibj2.command.PIDCommand;

import static frc.robot.Constants.SwerveConstants.*;

public class AutoAlignInPlaceCommand extends PIDCommand {

    @Override
    public void initialize() {
        Limelight.enable();
    }

    @Override
    public void end(boolean interrupted) {
        //Limelight.disable();
    }

    @Override
    public void execute() {
        super.execute();
        SmartDashboard.putNumber("TX",Limelight.getTx());
        SmartDashboard.putNumber("PError",getController().getPositionError());
        SmartDashboard.putNumber("VError",getController().getVelocityError());
        SmartDashboard.putNumber("isSetPoint",getController().atSetpoint() ? 10 : 0);
    }

    /**
     * Rotates SwerveDrive in place toward target until at tolerance.
     * Meant for Auto, Not meant for tuning / teleop.
     *
     * @param drivetrainSubsystem drivetrain instance
     */
    public AutoAlignInPlaceCommand(SwerveDrive drivetrainSubsystem) {

        super(new PIDController(SWERVE_TURRET_STATIONARY_KP, SWERVE_TURRET_STATIONARY_KI, SWERVE_TURRET_STATIONARY_KD),
                Limelight::getTx, //Measurement Source
                0, //Set point
                pidOutput -> { //Using output
                    drivetrainSubsystem.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                            0,
                            0,
                            pidOutput + Math.copySign(SWERVE_TURRET_STATIONARY_MIN, pidOutput),
                            drivetrainSubsystem.getGyroscopeRotation()
                    ));
                },
                drivetrainSubsystem);
        getController().enableContinuousInput(-180,180);
        getController().setTolerance(TURN_TOLERANCE, TURN_RATE_TOLERANCE);

    }

    @Override
    public boolean isFinished() {
        return getController().atSetpoint() && Limelight.getTx() != 0;
    }

}