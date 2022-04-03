package frc.robot.commands.drivetrain;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.hardware.Limelight;
import frc.robot.helper.auto.ThetaSupplier;
import frc.robot.helper.auto.UniformThetaSupplier;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.subsystems.SwerveDrive;
import jdk.jfr.Name;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.LEDConstants.AUTO_AIM_PATTERN;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.*;



public class AutoAlignDriveCommand extends CommandBase {
    private RobotLogger logger = new RobotLogger(AutoAlignDriveCommand.class.getCanonicalName());

    PIDController autoAlignPIDController;

    DoubleSupplier driverJoystickX;
    DoubleSupplier driverJoystickY;
    DoubleSupplier operatorJoystickX;

    SwerveDrive swerveDrive;

    /**
     * Continuously rotates swerve drive toward Limelight target.
     *
     * @param drivetrainSubsystem drivetrain instance
     * @param driverJoystickX Driver's Translation X
     * @param driverJoystickY Driver's Translation Y
     */

    public AutoAlignDriveCommand(SwerveDrive drivetrainSubsystem,
                                 DoubleSupplier driverJoystickX,
                                 DoubleSupplier driverJoystickY,
                                 DoubleSupplier operatorJoystickX) {

        this.swerveDrive = drivetrainSubsystem;

        this.driverJoystickX = driverJoystickX;
        this.driverJoystickY = driverJoystickY;

        this.operatorJoystickX = operatorJoystickX;

        addRequirements(drivetrainSubsystem);

    }

    public AutoAlignDriveCommand(SwerveDrive drivetrainSubsystem) {

        this.swerveDrive = drivetrainSubsystem;

        this.driverJoystickX = () -> 0;
        this.driverJoystickY = () -> 0;
        this.operatorJoystickX = () -> 0;

        addRequirements(drivetrainSubsystem);

    }

    //The angle between the hub and the robot
    public double angleBetweenHub(Pose2d robotPose) {
        if(HUB_POSITION_X - robotPose.getX() == 0){
            if(robotPose.getY() - HUB_POSITION_Y > 0){
                return 270;
            }
            else{
                return 90;
            }
        }
        return Math.atan(Math.abs(HUB_POSITION_Y - robotPose.getY())/Math.abs(HUB_POSITION_X - robotPose.getX()));
    }

    //The setpoint angle for the robot to turn towards the hub
    public double angleToHub(Pose2d robotPose) {
        double angleBetweenHub = Math.toDegrees(angleBetweenHub(robotPose));

        if(robotPose.getY() - HUB_POSITION_Y > 0){
            if(robotPose.getX() - HUB_POSITION_X > 0){
                angleBetweenHub+=180;
            }
            else if(robotPose.getX() - HUB_POSITION_X > 0){
                angleBetweenHub+=270;
            }
        }
        else if(robotPose.getY() - HUB_POSITION_Y < 0){
            if(robotPose.getX() - HUB_POSITION_X > 0){
                angleBetweenHub+=90;
            }
        }
        else if(angleBetweenHub == 0){
            if(robotPose.getX() - HUB_POSITION_X > 0){
                angleBetweenHub=180;
            }
            else if(robotPose.getX() - HUB_POSITION_X < 0){
                angleBetweenHub=0;
            }
        }

        return angleBetweenHub;
    }

    public void alignWithVision(){
        autoAlignPIDController = new PIDController(SWERVE_TURRET_KP, SWERVE_TURRET_KI, SWERVE_TURRET_KD);
        autoAlignPIDController.setSetpoint(0);
        autoAlignPIDController.enableContinuousInput(-180, 180);
    }

    public void alignWithoutVision(){

        autoAlignPIDController = new PIDController(SWERVE_TURRET_KP, SWERVE_TURRET_KI, SWERVE_TURRET_KD);
        autoAlignPIDController.setSetpoint(angleToHub(swerveDrive.getPose()));
        autoAlignPIDController.enableContinuousInput(0, 360);
    }

    @Override
    public void execute() {
        double autoAlignPidOutput;

        if(Limelight.isTargetDetected()){
            alignWithVision();
            autoAlignPidOutput = autoAlignPIDController.calculate(Limelight.getTx());
        }
        else{
            alignWithoutVision();
            autoAlignPidOutput = autoAlignPIDController.calculate(swerveDrive.getPose().getRotation().getDegrees());
        }

        //Save some Computation from Sqrt
        double speedSquared = Math.pow(driverJoystickX.getAsDouble(), 2) + Math.pow(driverJoystickY.getAsDouble(),2);

        // Use translation from Driver, Rotation is from PID
        // Ternary Explanation: Since while moving we can easily rotate, we don't mess with the PID
        // but while NOT moving, the motors need more power in order to actually move, so
        // we add a Constant, (Using copysign to either add or subtract depending on sign)
        double autoAlignPIDRotationalOutput = speedSquared > Math.pow(0.1, 2) ?
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
    public void initialize() {
        Limelight.enable();
        AUTO_AIM_PATTERN.update(true);
        logger.info("Auto Align Enabled");
    }

    @Override
    public void end(boolean interrupted) {
        Limelight.disable();
        AUTO_AIM_PATTERN.update(false);
    }
}