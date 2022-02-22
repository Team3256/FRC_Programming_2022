package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.helper.Limelight;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.CANdleConstants.AUTO_AIM_PATTERN;
import static frc.robot.Constants.CANdleConstants.LEDSectionName.AUTO_AIM;
import static frc.robot.Constants.CANdleConstants.SECTIONS;
import static frc.robot.Constants.SwerveConstants.*;

public class AutoAlignDriveContinuousCommand extends PIDCommand {
    /**
     * Continuously rotates swerve drive toward Limelight target. Use tuningSetup() for easy tuning.
     *
     * @param drivetrainSubsystem drivetrain instance
     * @param joystickX Driver's Translation X
     * @param joystickY Driver's Translation Y
     */
    public AutoAlignDriveContinuousCommand (SwerveDrive drivetrainSubsystem,
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

                    //Tuning
                    if (IS_TUNING_SWERVE_TURRET) {
                        double current_distance = Limelight.getTunedDistanceToTarget();
                        double current_velocity = drivetrainSubsystem.getVelocity().getTranslation().getNorm();
                        SmartDashboard.putNumber("Swerve Velocity", current_velocity);
                        SmartDashboard.putNumber("Distance to target", current_distance);
                        SmartDashboard.putNumber("Swerve Turret PID OUT", motorOutput);
                        SmartDashboard.putNumber("Swerve Turret Limelight TX", Limelight.getTx());
                    }
                },
                drivetrainSubsystem);
        getController().enableContinuousInput(-180,180);
        if (IS_TUNING_SWERVE_TURRET) {
            SmartDashboard.putData("Swerve Turret PID", getController());
        }

    }

    public static void tuningSetup(){
        // not sure if we need this
//        SmartDashboard.setDefaultNumber("Swerve Turret kP", 0);
//        SmartDashboard.setDefaultNumber("Swerve Turret kI", 0);
//        SmartDashboard.setDefaultNumber("Swerve Turret kD", 0);
//        SmartDashboard.setDefaultNumber("Swerve Turret Stationary Min", 0);
//
//        SWERVE_TURRET_KP = SmartDashboard.getNumber("Swerve Turret kP", 0);
//        SWERVE_TURRET_KI = SmartDashboard.getNumber("Swerve Turret kI", 0);
//        SWERVE_TURRET_KD = SmartDashboard.getNumber("Swerve Turret kD", 0);
//        SWERVE_TURRET_STATIONARY_MIN = SmartDashboard.getNumber("Swerve Turret Stationary Min", 0);
    }


    @Override
    public boolean isFinished() {
        return false;
    }

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
}