package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hardware.Limelight;
import frc.robot.helper.AdaptiveSlewRateLimiter;
import frc.robot.helper.logging.RobotLogger;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Log;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.LimelightAutoCorrectConstants.MAX_VISION_LOCALIZATION_HEADING_CORRECTION;
import static frc.robot.Constants.LimelightAutoCorrectConstants.MAX_VISION_LOCALIZATION_TRANSLATION_CORRECTION;
import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.FieldConstants.*;


public class SwerveDrive extends SubsystemBase implements Loggable {
    private static final RobotLogger logger = new RobotLogger(SwerveDrive.class.getCanonicalName());
    private final AdaptiveSlewRateLimiter adaptiveXRateLimiter = new AdaptiveSlewRateLimiter(X_ACCEL_RATE_LIMIT, X_DECEL_RATE_LIMIT);
    private final AdaptiveSlewRateLimiter adaptiveYRateLimiter = new AdaptiveSlewRateLimiter(Y_ACCEL_RATE_LIMIT, Y_DECEL_RATE_LIMIT);
    public static final double MAX_VOLTAGE = 12.0;
    private static final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            // Front Right
            new Translation2d(DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(DRIVETRAIN_TRACK_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Front left
            new Translation2d(-DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
            // Back right
            new Translation2d(-DRIVETRAIN_TRACK_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0)
    );

    private final WPI_Pigeon2 pigeon = new WPI_Pigeon2(DRIVETRAIN_PIGEON_ID);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private Pose2d pose = new Pose2d(0, 0, new Rotation2d(0));
    private Pose2d curr_velocity = new Pose2d();
    private final Field2d field = new Field2d();
    private SwerveDrivePoseEstimator poseEstimator;

    public SwerveDrive() {
        pigeon.configMountPoseYaw(GYRO_YAW_OFFSET);

        poseEstimator = new SwerveDrivePoseEstimator(getGyroscopeRotation(), new Pose2d(), kinematics,
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.012, 0.012, 0.01), // Current state X, Y, theta.
                new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.008), // Gyro reading theta stdevs
                new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.11, 0.11, 0.05) // Vision stdevs X, Y, and theta.
        );
//            new SwerveDrivePoseEstimator(kinematics, getGyroscopeRotation(), pose);

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_ENCODER_ID,
                -FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_ENCODER_ID,
                -FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
                BACK_LEFT_MODULE_STEER_MOTOR_ID,
                BACK_LEFT_MODULE_STEER_ENCODER_ID,
                -BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L2,
                BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
                BACK_RIGHT_MODULE_STEER_MOTOR_ID,
                BACK_RIGHT_MODULE_STEER_ENCODER_ID,
                -BACK_RIGHT_MODULE_STEER_OFFSET
        );
        logger.info("Swerve Drive Modules Initialized");
    }

    public void zeroGyroscope() {
        pigeon.reset();
        resetOdometry(new Pose2d());
        logger.info("zeroed gyroscope");
    }

    // @Log.Gyro(name = "Gyro Angle")
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void resetOdometry(Pose2d pose) {
        poseEstimator.resetPosition(pose, getGyroscopeRotation());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds.omegaRadiansPerSecond = INVERT_TURN ? -chassisSpeeds.omegaRadiansPerSecond : chassisSpeeds.omegaRadiansPerSecond;
        chassisSpeeds.vxMetersPerSecond = adaptiveXRateLimiter.calculate(chassisSpeeds.vxMetersPerSecond);
        chassisSpeeds.vyMetersPerSecond = adaptiveYRateLimiter.calculate(chassisSpeeds.vyMetersPerSecond);
        this.chassisSpeeds = chassisSpeeds;
    }

    public Pose2d getPose() { return poseEstimator.getEstimatedPosition();}


    public double getEstimatedDistance(){
        return Math.hypot(poseEstimator.getEstimatedPosition().getX(), poseEstimator.getEstimatedPosition().getY());
    }

    /**
     * @param distanceToTarget Distance to target in meters
     * @param thetaTargetOffset Limelight angle error in degrees
     */
    public void limelightLocalization(double limelightDistanceToTarget, double thetaTargetOffset) {
        Pose2d currentPose = getPose(); 
        Translation2d hubCenteredRobotPosition = currentPose.getTranslation().minus(HUB_POSITION); // coordinates with hub as origin

        double theta = Math.atan2(-hubCenteredRobotPosition.getY(), -hubCenteredRobotPosition.getX());
        if(theta < 0) theta += 2*Math.PI;
        Rotation2d robotCorrectedHeading = new Rotation2d(theta + Math.toRadians(thetaTargetOffset));

        double distanceToTarget = limelightDistanceToTarget + Constants.FieldConstants.UPPER_HUB_RADIUS;
        Translation2d visionTranslationHubCentered = new Translation2d(distanceToTarget * Math.cos(theta), distanceToTarget * Math.sin(theta));
        Translation2d visionTranslation = visionTranslationHubCentered.plus(Constants.FieldConstants.HUB_POSITION);

        Pose2d visionPose = new Pose2d(visionTranslation, robotCorrectedHeading);

        if (
                Math.abs(currentPose.relativeTo(visionPose).getTranslation().getNorm()) <= MAX_VISION_LOCALIZATION_TRANSLATION_CORRECTION &&
                Math.abs(currentPose.getRotation().minus(robotCorrectedHeading).getDegrees()) <= MAX_VISION_LOCALIZATION_HEADING_CORRECTION
        ){
            // only update position if measurement is not obviously wrong
            SwerveDrive.logger.info("Updating Pose Based off vision");
            poseEstimator.addVisionMeasurement(visionPose, Timer.getFPGATimestamp());
        } else {
            SwerveDrive.logger.info("Not Updating Pose Based off vision");
        }
    }

    public Pose2d getVelocity() { return curr_velocity; }

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

        SwerveModuleState frontLeftOptimized = optimizeModuleState(desiredStates[0], frontLeftModule.getSteerAngle());
        SwerveModuleState frontRightOptimized = optimizeModuleState(desiredStates[1], frontRightModule.getSteerAngle());
        SwerveModuleState backLeftOptimized = optimizeModuleState(desiredStates[2], backLeftModule.getSteerAngle());
        SwerveModuleState backRightOptimized = optimizeModuleState(desiredStates[3], backRightModule.getSteerAngle());

        if (Constants.DEBUG) {
            debugSwerveOffsets();
        }
      
        frontLeftModule.set(
                deadzoneMotor(frontLeftOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE),
                willDeadzoneMotor(frontLeftOptimized.speedMetersPerSecond) ? frontLeftModule.getSteerAngle() : frontLeftOptimized.angle.getRadians()
        );

        frontRightModule.set(
                deadzoneMotor(frontRightOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE),
                willDeadzoneMotor(frontRightOptimized.speedMetersPerSecond) ? frontRightModule.getSteerAngle() : frontRightOptimized.angle.getRadians()
        );

        backRightModule.set(
                deadzoneMotor(backRightOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE),
                willDeadzoneMotor(backRightOptimized.speedMetersPerSecond) ? backRightModule.getSteerAngle() : backRightOptimized.angle.getRadians()
        );

        backLeftModule.set(
                deadzoneMotor(backLeftOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE),
                willDeadzoneMotor(backLeftOptimized.speedMetersPerSecond) ? backLeftModule.getSteerAngle() : backLeftOptimized.angle.getRadians()
        );
    }

    private boolean willDeadzoneMotor(double speed) {
        return Math.abs(speed / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE) < DRIVETRAIN_MOTOR_DEADZONE_VOLTS;
    }

    private double deadzoneMotor(double volts) {
        if (Math.abs(volts) < DRIVETRAIN_MOTOR_DEADZONE_VOLTS) {
            return 0;
        }

        final double SIGNED_DEADZONE = Math.copySign(DRIVETRAIN_MOTOR_DEADZONE_VOLTS, volts);
        return (((MAX_VOLTAGE - SIGNED_DEADZONE)/MAX_VOLTAGE) * volts) + Math.copySign(SIGNED_DEADZONE, volts);
    }

    public void setTrajectory(Trajectory trajectory) {
        field.getObject("traj").setTrajectory(trajectory);
    } // only used for debug on Glass, not for following trajectories

    public void outputToDashboard() {
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Front Left Speed", frontLeftModule.getDriveVelocity());
            SmartDashboard.putNumber("Front Right Speed", frontRightModule.getDriveVelocity());
            SmartDashboard.putNumber("Back Left Speed", backLeftModule.getDriveVelocity());
            SmartDashboard.putNumber("Back Right Speed", backRightModule.getDriveVelocity());

            SmartDashboard.putNumber("Position in Inches", Units.metersToInches(pose.getTranslation().getX()));

            SmartDashboard.putNumber("Gyro Rotation", pose.getRotation().getDegrees());

            field.setRobotPose(getPose());
            SmartDashboard.putData("Field", field);
        }
    }

    public SwerveModuleState optimizeModuleState(SwerveModuleState state, double heading) {
        return SwerveModuleState.optimize(state, new Rotation2d(heading));
    }

    @Override
    public void periodic() {
        double timestamp = Timer.getFPGATimestamp();
        limelightLocalization(Limelight.getRawDistanceToTarget(), Limelight.getTx());

        Rotation2d gyroAngle = getGyroscopeRotation();
        // Update the pose
        SwerveModuleState frontLeftState = new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle()));
        SwerveModuleState frontRightState = new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle()));
        SwerveModuleState backLeftState = new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle()));
        SwerveModuleState backRightState = new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()));
        pose = poseEstimator.updateWithTime(timestamp, gyroAngle, frontRightState, backRightState,
                frontLeftState, backLeftState);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);

        outputToDashboard();
    }

    public void debugSwerveOffsets() {
        SmartDashboard.putNumber("Front Left Swerve Module Standard Offset: ", frontLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Front Right Swerve Module Standard Offset: ", frontRightModule.getSteerAngle());
        SmartDashboard.putNumber("Back Left Swerve Module Standard Offset: ", backLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Back Right Swerve Module Standard Offset: ", backRightModule.getSteerAngle());
    }
}
