// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.logging.Level;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public static class SwerveConstants {
        public static final double MAX_VOLTAGE = 12.0;

        public static final double DRIVETRAIN_TRACK_METERS = 0.4445;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168.8379);
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(233.1738);
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(349.8926);
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(52.8223);

        private static final double MOTOR_FREE_SPIN_RPM = 6380.0;

        // Calculated Values (Don't Change)
        public static final double MAX_VELOCITY_METERS_PER_SECOND = MOTOR_FREE_SPIN_RPM / 60.0 *
                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
    }
    public static class AutoConstants {
        public static double MIN_SPACE_BETWEEN_POINTS = 0.5;
        public static final double[] FRONT_LEFT = {0.5, 0.5};
        public static final double[] FRONT_RIGHT = {0.5,-0.5};
        public static final double[] BACK_LEFT = {-0.5,0.5};
        public static final double[] BACK_RIGHT = {-0.5,-0.5};
        public static double P_THETA_CONTROLLER;
        public static double kMaxSpeedMetersPerSecond;
        public static double kMaxAccelerationMetersPerSecondSquared;
        public static double kPXController;
        public static TrapezoidProfile.Constraints kThetaControllerConstraints;
        public static double kPYController;
    }

    public static class IDConstants {
        public static final int[] TALON_FX_IDS = new int[]{5,6,8,9,11,12,14,15,20,21};
        public static final int[] SPARK_MAX_IDS = new int[]{};

        public static final int DRIVETRAIN_PIGEON_ID = 4;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 5;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 6;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 7;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 8;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 9;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 10;

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 11;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 12;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 13;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 14;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 15;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 16;

        public static final int PID_SHOOTER_MOTOR_ID_LEFT = 7;
        public static final int PID_SHOOTER_MOTOR_ID_RIGHT = 8;

        // Channels

        public static final int HOOD_SERVO_CHANNEL_ID = 0;


    }
    public static class LoggingConstants {


        // ******* DEBUG SETTINGS ******* //

        //Make sure this is WARNING in Competition
        //Ideally CONSOLE should be at most warning, too cluttered otherwise
        public static Level CONSOLE_LEVEL = Level.WARNING;

        //Make sure this is INFO in Competition
        public static Level LOG_LEVEL = Level.INFO;

        //Uses Internal Storage for normal logging, be careful of
        //using too much storage - Normally should be false.
        public static boolean FORCE_NORMAL_INTERNAL = false;

        // ******************************* //


        //Max Number of Files
        public static final int TXT_LOG_MAX_FILES = 10;
        public static final int HTML_LOG_MAX_FILES = 3;

        //Normal Max File Sizes
        public static final int TXT_LOG_MAX_SIZE = 100000000;  // In Bytes
        public static final int HTML_LOG_MAX_SIZE = 100000000; // In Bytes

        //File Names (%g is for Numbering of Files)
        public static final String TXT_FILE_NAME = "TextLog%g.txt";
        public static final String HTML_FILE_NAME = "HtmlLog%g.html";


        //Emergency File Settings (NO USB)
        public static final int EMERGENCY_TXT_MAX_FILES = 3;
        public static final int EMERGENCY_TXT_MAX_SIZE = 3000000; //In Bytes


    }

    public static class ShooterConstants {
        // Constant Shooting Section
        public static final double RADIUS_UPPER_HUB = 0.61; // in m
        public static final double SHOOTER_HEIGHT = 0.51; // in m
        public static final double UPPER_HUB_AIMING_HEIGHT = 2.725427; // in m

        // Tuning Section
        public static final double DELTA_AIM_HEIGHT_FACTOR = 0.0; // TODO: Set delta aim height factor from tuning
        public static final double DELTA_DISTANCE_TO_TARGET_FACTOR = 0.0; // TODO: Set delta distance from tuning
        public static final double SET_POINT_ERROR_MARGIN = 0.0; // in m/s TODO: Set margin of error for initiation speed test
        public static final double OFFSET_HEIGHT_FACTOR = 0.0; // TODO: From tuning, set offset height
        public static final double OFFSET_DISTANCE_FACTOR = 0.0; // TODO: From tuning, set offset distance

        // Hood Angle Constants
        public static final double HOOD_ANGLE_UPPER_LIMIT = (75 * Math.PI / 180.0);
        public static final double HOOD_ANGLE_LOWER_LIMIT = (45 * Math.PI / 180.0);
    }
}
