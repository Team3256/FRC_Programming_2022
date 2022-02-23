package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.SwerveConstants.*;

public class AutoAlignDriveContinuousCommand extends CommandBase {


    PIDController autoAlignPIDController;

    DoubleSupplier driverJoystickX;
    DoubleSupplier driverJoystickY;
    DoubleSupplier operatorJoystickX;
    DoubleSupplier operatorJoystickY;

    SwerveDrive swerveDrive;


    /**
     * Continuously rotates swerve drive toward Limelight target.
     *
     * @param drivetrainSubsystem drivetrain instance
     * @param driverJoystickX Driver's Translation X
     * @param driverJoystickY Driver's Translation Y
     */
    public AutoAlignDriveContinuousCommand (SwerveDrive drivetrainSubsystem,
                                            DoubleSupplier driverJoystickX,
                                            DoubleSupplier driverJoystickY,
                                            DoubleSupplier operatorJoystickX) {

        this.swerveDrive = drivetrainSubsystem;

        this.driverJoystickX = driverJoystickX;
        this.driverJoystickY = driverJoystickY;

        this.operatorJoystickX = operatorJoystickX;

        autoAlignPIDController = new PIDController(SWERVE_TURRET_KP, SWERVE_TURRET_KI, SWERVE_TURRET_KD);
        autoAlignPIDController.setSetpoint(0);
        autoAlignPIDController.enableContinuousInput(-180,180);
    }

    @Override
    public void initialize() {
        Limelight.enable();
    }

    @Override
    public void execute() {

        double autoAlignPidOutput = autoAlignPIDController.calculate(Limelight.getTx());

        //Save some Computation from Sqrt
        double speedSquared = Math.pow(driverJoystickX.getAsDouble(),2) + Math.pow(driverJoystickY.getAsDouble(),2);

        // Use translation from Driver, Rotation is from PID
        // Ternary Explanation: Since while moving we can easily rotate, we don't mess with the PID
        // but while NOT moving, the motors need more power in order to actually move, so
        // we add a Constant, (Using copysign to either add or subtract depending on sign)
        double autoAlignPIDRotationalOutput = 0;
        if (Math.abs(operatorJoystickX.getAsDouble()) > SWERVE_TURRET_OPERATOR_DEADZONE ||
                Math.abs(operatorJoystickY.getAsDouble()) > SWERVE_TURRET_OPERATOR_DEADZONE)
            autoAlignPIDRotationalOutput = speedSquared > Math.pow(0.1,2) ?
                    autoAlignPidOutput :
                    autoAlignPidOutput + Math.copySign(SWERVE_TURRET_STATIONARY_MIN, autoAlignPidOutput);

        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                driverJoystickX.getAsDouble(),
                driverJoystickY.getAsDouble(),
                autoAlignPIDRotationalOutput +
                        (SWERVE_TURRET_OPERATOR_INFLUENCE*operatorJoystickX.getAsDouble()),
                swerveDrive.getGyroscopeRotation()
        ));
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.disable();
    }
}