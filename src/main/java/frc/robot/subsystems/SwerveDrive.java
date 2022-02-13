package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2_Faults;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
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

import java.util.logging.Logger;

import static frc.robot.Constants.SwerveConstants.*;
import static frc.robot.Constants.IDConstants.*;


public class SwerveDrive extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(SwerveDrive.class.getCanonicalName());

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
    private static Pigeon2_Faults pigeonFaults = new Pigeon2_Faults();

    private final SwerveModule frontLeftModule;
    private final SwerveModule frontRightModule;
    private final SwerveModule backLeftModule;
    private final SwerveModule backRightModule;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private Pose2d pose = new Pose2d(0, 0, new Rotation2d(0));
    private SwerveDriveOdometry odometry = new SwerveDriveOdometry(kinematics, getGyroscopeRotation(), pose);

    private boolean highAccDetectedPrev = false;

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

    public String getFaultMessage() {
        if(!pigeonFaults.hasAnyFault()) return "No faults";
        String retval = "";
        retval += pigeonFaults.APIError ? "APIError, " : "";
        retval += pigeonFaults.AccelFault ? "AccelFault, " : "";
        retval += pigeonFaults.BootIntoMotion ? "BootIntoMotion, " : "";
        retval += pigeonFaults.GyroFault ? "GyroFault, " : "";
        retval += pigeonFaults.HardwareFault ? "HardwareFault, " : "";
        retval += pigeonFaults.MagnetometerFault ? "MagnetometerFault, " : "";
        retval += pigeonFaults.ResetDuringEn ? "ResetDuringEn, " : "";
        retval += pigeonFaults.SaturatedAccel ? "SaturatedAccel, " : "";
        retval += pigeonFaults.SaturatedMag ? "SaturatedMag, " : "";
        retval += pigeonFaults.SaturatedRotVelocity ? "SaturatedRotVelocity, " : "";
        logger.warning("Pigeon Error: "+ retval);
        return retval;
    }

    public SwerveDriveKinematics getKinematics() {
        return kinematics;
    }

    public void resetOdometry(Pose2d pose) {
        odometry.resetPosition(pose, getGyroscopeRotation());
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

        SmartDashboard.putNumber("Gyro Rotation", pose.getRotation().getDegrees());
        SmartDashboard.putString("Gyro Errors", getFaultMessage());

    }

    public SwerveModuleState optimizeModuleState(SwerveModuleState state, double heading) {
        return SwerveModuleState.optimize(state, new Rotation2d(heading));
    }

    @Override
    public void periodic() {

        //Log any High Acceleration Events, bool variable to ensure 1 log per real event
        short[] acc = new short[3];
        pigeon.getBiasedAccelerometer(acc);
        double squaredAcc = acc[0] * acc[0] + acc[1] * acc[1];
        if (!highAccDetectedPrev && squaredAcc > Math.pow(2 * 16384,2)) {
            logger.warning("High Acceleration Detected: " + Math.sqrt(squaredAcc));
            highAccDetectedPrev = true;
        } else {
            highAccDetectedPrev = false;
        }

        Rotation2d gyroAngle = getGyroscopeRotation();
        pigeon.getFaults(pigeonFaults);
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