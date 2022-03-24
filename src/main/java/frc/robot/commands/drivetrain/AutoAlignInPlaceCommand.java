package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Limelight;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.SwerveDrive;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.LEDConstants.AUTO_AIM_PATTERN;
import static frc.robot.Constants.SwerveConstants.*;

public class AutoAlignInPlaceCommand extends CommandBase {
    private RobotLogger logger = new RobotLogger(AutoAlignDriveContinuousCommand.class.getCanonicalName());

    PIDController autoAlignPIDController;

    SwerveDrive swerveDrive;


    /**
     * Continuously rotates swerve drive toward Limelight target.
     *
     * @param drivetrainSubsystem drivetrain instance
     */
    public AutoAlignInPlaceCommand (SwerveDrive drivetrainSubsystem){

        this.swerveDrive = drivetrainSubsystem;

        autoAlignPIDController = new PIDController(SWERVE_TURRET_KP, SWERVE_TURRET_KI, SWERVE_TURRET_KD);
        autoAlignPIDController.setSetpoint(0);
        autoAlignPIDController.enableContinuousInput(-180,180);

        addRequirements(drivetrainSubsystem);

    }



    @Override
    public void execute() {

        double autoAlignPidOutput = autoAlignPIDController.calculate(Limelight.getTx());

        //Save some Computation from Sqrt

        // Use translation from Driver, Rotation is from PID
        // Ternary Explanation: Since while moving we can easily rotate, we don't mess with the PID
        // but while NOT moving, the motors need more power in order to actually move, so
        // we add a Constant, (Using copysign to either add or subtract depending on sign)
        double autoAlignPIDRotationalOutput = autoAlignPidOutput + Math.copySign(SWERVE_TURRET_STATIONARY_MIN, autoAlignPidOutput);

        swerveDrive.drive(ChassisSpeeds.fromFieldRelativeSpeeds(
                0,
                0,
                autoAlignPIDRotationalOutput,
                swerveDrive.getGyroscopeRotation()
        ));
    }


    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void initialize() {
        Limelight.enable();
        logger.info("Auto Align In Place Enabled");
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.disable();
    }
}