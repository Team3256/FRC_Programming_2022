package frc.robot.subsystems;

import com.ctre.phoenix.sensors.PigeonIMU;
import com.swervedrivespecialties.swervelib.Mk3SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SwerveModule;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.IDConstants.*;


public class SwerveDrive extends SubsystemBase {
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
    private final PigeonIMU pigeon = new PigeonIMU(DRIVETRAIN_PIGEON_ID);

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private Pose2d pose = new Pose2d(0, 0, new Rotation2d(0));
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), pose);

    public SwerveDrive() {
        ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");

        // FIXME Setup motor configuration
        frontLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                // This parameter is optional, but will allow you to see the current state of the module on the dashboard.
                tab.getLayout("Front Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(0, 0),
                // This can either be STANDARD or FAST depending on your gear configuration
                Mk3SwerveModuleHelper.GearRatio.FAST,
                FRONT_LEFT_MODULE_DRIVE_MOTOR,
                FRONT_LEFT_MODULE_STEER_MOTOR,
                FRONT_LEFT_MODULE_STEER_ENCODER_ID,
                FRONT_LEFT_MODULE_STEER_OFFSET
        );

        frontRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Front Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(2, 0),
                Mk3SwerveModuleHelper.GearRatio.FAST,
                FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                FRONT_RIGHT_MODULE_STEER_MOTOR,
                FRONT_RIGHT_MODULE_STEER_ENCODER_ID,
                FRONT_RIGHT_MODULE_STEER_OFFSET
        );

        backLeftModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Left Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(4, 0),
                Mk3SwerveModuleHelper.GearRatio.FAST,
                BACK_LEFT_MODULE_DRIVE_MOTOR,
                BACK_LEFT_MODULE_STEER_MOTOR,
                BACK_LEFT_MODULE_STEER_ENCODER_ID,
                BACK_LEFT_MODULE_STEER_OFFSET
        );

        backRightModule = Mk3SwerveModuleHelper.createFalcon500(
                tab.getLayout("Back Right Module", BuiltInLayouts.kList)
                        .withSize(2, 4)
                        .withPosition(6, 0),
                Mk3SwerveModuleHelper.GearRatio.FAST,
                BACK_RIGHT_MODULE_DRIVE_MOTOR,
                BACK_RIGHT_MODULE_STEER_MOTOR,
                BACK_RIGHT_MODULE_STEER_ENCODER_ID,
                BACK_RIGHT_MODULE_STEER_OFFSET
        );
    }

    /**
     * Sets the gyroscope angle to zero. This can be used to set the direction the robot is currently facing to the
     * 'forwards' direction.
     */
    public void zeroGyroscope() {
        pigeon.setFusedHeading(0.0);
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getGyroscopeRotation());
    }

    public Rotation2d getGyroscopeRotation() {
        return Rotation2d.fromDegrees(pigeon.getFusedHeading());
    }

    public void drive(ChassisSpeeds chassisSpeeds) {
        this.chassisSpeeds = chassisSpeeds;
    }

    public Pose2d getPose() { return odometry.getPoseMeters();}

    /**
     * Sets the swerve ModuleStates.
     *
     * @param desiredStates The desired SwerveModule states.
     */
    public void setModuleStates(SwerveModuleState[] desiredStates) {
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_VELOCITY_METERS_PER_SECOND);

        SmartDashboard.putNumber("Desired Front Left Speed", desiredStates[0].speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Front Right Speed", desiredStates[1].speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Back Left Speed", desiredStates[2].speedMetersPerSecond);
        SmartDashboard.putNumber("Desired Back Right Speed", desiredStates[3].speedMetersPerSecond);

        SmartDashboard.putNumber("Desired Front Left Angle", desiredStates[0].angle.getDegrees());
        SmartDashboard.putNumber("Desired Front Right Angle", desiredStates[1].angle.getDegrees());
        SmartDashboard.putNumber("Desired Back Left Angle", desiredStates[2].angle.getDegrees());
        SmartDashboard.putNumber("Desired Back Right Angle", desiredStates[3].angle.getDegrees());


        SwerveModuleState frontLeftOptimized = optimizeModuleState(desiredStates[0], frontLeftModule.getSteerAngle());
        SwerveModuleState frontRightOptimized = optimizeModuleState(desiredStates[1], frontRightModule.getSteerAngle());
        SwerveModuleState backLeftOptimized = optimizeModuleState(desiredStates[2], backLeftModule.getSteerAngle());
        SwerveModuleState backRightOptimized = optimizeModuleState(desiredStates[3], backRightModule.getSteerAngle());

        frontLeftModule.set(frontLeftOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, frontLeftOptimized.angle.getRadians());
        frontRightModule.set(frontRightOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, frontRightOptimized.angle.getRadians());
        backLeftModule.set(backLeftOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, backLeftOptimized.angle.getRadians());
        backRightModule.set(backRightOptimized.speedMetersPerSecond / MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE, backRightOptimized.angle.getRadians());
    }

    public void outputToDashboard() {

        SmartDashboard.putNumber("Front Left Speed", frontLeftModule.getDriveVelocity());
        SmartDashboard.putNumber("Front Right Speed", frontRightModule.getDriveVelocity());
        SmartDashboard.putNumber("Back Left Speed", backLeftModule.getDriveVelocity());
        SmartDashboard.putNumber("Back Right Speed", backRightModule.getDriveVelocity());

        SmartDashboard.putNumber("Front Left Angle", frontLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Front Right Angle", frontRightModule.getSteerAngle());
        SmartDashboard.putNumber("Back Left Angle", backLeftModule.getSteerAngle());
        SmartDashboard.putNumber("Back Right Angle", backRightModule.getSteerAngle());
        SmartDashboard.putNumber("Position in Inches", Units.metersToInches(pose.getTranslation().getX()));


//        short[] r = new short[3];
//        pigeon.getBiasedAccelerometer(r);
//        SmartDashboard.putNumber("Gyro Acceleration", r[2]);
    }

    public SwerveModuleState optimizeModuleState(SwerveModuleState state, double heading) {
        return SwerveModuleState.optimize(state, new Rotation2d(heading));
    }

    @Override
    public void periodic() {
        Rotation2d gyroAngle = getGyroscopeRotation();
        // Update the pose
        SwerveModuleState frontLeftState = new SwerveModuleState(frontLeftModule.getDriveVelocity(), new Rotation2d(frontLeftModule.getSteerAngle()));
        SwerveModuleState frontRightState = new SwerveModuleState(frontRightModule.getDriveVelocity(), new Rotation2d(frontRightModule.getSteerAngle()));
        SwerveModuleState backLeftState = new SwerveModuleState(backLeftModule.getDriveVelocity(), new Rotation2d(backLeftModule.getSteerAngle()));
        SwerveModuleState backRightState = new SwerveModuleState(backRightModule.getDriveVelocity(), new Rotation2d(backRightModule.getSteerAngle()));

        pose = odometry.update(gyroAngle, frontRightState, backRightState,
                frontLeftState, backLeftState);

        SwerveModuleState[] states = kinematics.toSwerveModuleStates(chassisSpeeds);
        setModuleStates(states);

        outputToDashboard();
    }
}