package frc.robot.subsystems;

import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.helper.SmoothVelocity;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.Constants;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.IDConstants.*;
import static frc.robot.helper.SmoothVelocity.smoothVelocity;


public class SwerveDrive extends SubsystemBase {
    private static final RobotLogger logger = new RobotLogger(SwerveDrive.class.getCanonicalName());

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
    private double last_timestamp = Timer.getFPGATimestamp();
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), pose);
    private boolean highAccDetectedPrev = false;

    public SwerveDrive() {
        pigeon.configMountPoseYaw(GYRO_YAW_OFFSET);

        frontLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L4,
                FRONT_LEFT_MODULE_DRIVE_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_MOTOR_ID,
                FRONT_LEFT_MODULE_STEER_ENCODER_ID,
                FRONT_LEFT_MODULE_STEER_OFFSET + 90
        );

        frontRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L4,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_MOTOR_ID,
                FRONT_RIGHT_MODULE_STEER_ENCODER_ID,
                FRONT_RIGHT_MODULE_STEER_OFFSET + 180
        );

        backLeftModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L4,
                BACK_LEFT_MODULE_DRIVE_MOTOR_ID,
                BACK_LEFT_MODULE_STEER_MOTOR_ID,
                BACK_LEFT_MODULE_STEER_ENCODER_ID,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk4SwerveModuleHelper.createFalcon500(
                Mk4SwerveModuleHelper.GearRatio.L4,
                BACK_RIGHT_MODULE_DRIVE_MOTOR_ID,
                BACK_RIGHT_MODULE_STEER_MOTOR_ID,
                BACK_RIGHT_MODULE_STEER_ENCODER_ID,
                BACK_RIGHT_MODULE_STEER_OFFSET - 90
        );
        logger.info("Swerve Drive Modules Initialized");
    }

    public void zeroGyroscope() {
        pigeon.reset();
        resetOdometry(new Pose2d());
        logger.info("zeroed gyroscope");
    }
    
    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getYaw());
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getGyroscopeRotation());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        chassisSpeeds.omegaRadiansPerSecond = INVERT_TURN ? -chassisSpeeds.omegaRadiansPerSecond : chassisSpeeds.omegaRadiansPerSecond;
        this.chassisSpeeds = chassisSpeeds;

    }

    public Pose2d getPose() { return odometry.getPoseMeters();}

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
    }

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
        double dt = timestamp - last_timestamp;
        last_timestamp = timestamp;

        Rotation2d gyroAngle = getGyroscopeRotation();
        // Update the pose
        SwerveModuleState frontLeftState = new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle()));
        SwerveModuleState frontRightState = new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle()));
        SwerveModuleState backLeftState = new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle()));
        SwerveModuleState backRightState = new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()));

        Pose2d lastPose = pose;
        pose = odometry.update(gyroAngle, frontRightState, backRightState,
                frontLeftState, backLeftState);
        Pose2d diff = lastPose.relativeTo(pose);
        curr_velocity = new Pose2d(
                new Translation2d(diff.getX() / dt, diff.getY() / dt),
                new Rotation2d(diff.getRotation().getRadians() / dt)
        );

//       chassisSpeeds.vxMetersPerSecond =  smoothVelocity(curr_velocity.getTranslation().getX(), chassisSpeeds.vxMetersPerSecond, MAX_ACCELERATION, dt);
//       chassisSpeeds.vyMetersPerSecond =  smoothVelocity(curr_velocity.getTranslation().getY(), chassisSpeeds.vyMetersPerSecond, MAX_ACCELERATION, dt);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);

        outputToDashboard();
    }

    public void forward(double meters){
        drive(new ChassisSpeeds(0,meters,0));
    }

    public void backward(double meters){
        drive(new ChassisSpeeds(0,-meters,0));
    }

    public void pivotTurn(double rad){
        drive( new ChassisSpeeds(0,0,rad));
    }

    public void fixedRightRotate(int volts){
        frontLeftModule.set(deadzoneMotor(volts), 0);
        backLeftModule.set(deadzoneMotor(volts), 0);
    }
    public void fixedLeftRotate(int volts){
        frontRightModule.set(deadzoneMotor(volts), 0);
        backRightModule.set(deadzoneMotor(volts), 0);
    }
    public void stop(){
        drive(new ChassisSpeeds(0,0,0));
    }


    public void debugSwerveOffsets() {
        SmartDashboard.putNumber("Front Left Swerve Module Standard Offset: ", Math.toDegrees(frontLeftModule.getSteerAngle()) - 90);
        SmartDashboard.putNumber("Front Right Swerve Module Standard Offset: ", Math.toDegrees(frontRightModule.getSteerAngle()) - 180);
        SmartDashboard.putNumber("Back Left Swerve Module Standard Offset: ", Math.toDegrees(backLeftModule.getSteerAngle()));
        SmartDashboard.putNumber("Back Right Swerve Module Standard Offset: ", Math.toDegrees(backRightModule.getSteerAngle()) + 90);
    }
}