package frc.robot;

import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

import java.util.logging.Level;

public final class Constants {
    public static class SwerveConstants {
        public static final double DRIVETRAIN_TRACK_METERS = 0.4445;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

        public static final int DRIVETRAIN_PIGEON_ID = 4; // FIXME get a pigeon lmao

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR = 5;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR = 6;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER = 7;
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168.8379); //357

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR = 8;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR = 9;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER = 10;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(233.1738); //179

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR = 11;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR = 12;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER = 13;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(349.8926);

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR = 14;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR = 15;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER = 16;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(52.8223); //179

        public static final double MAX_METERS_PER_SECOND = 10;
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 1380.0 / 60.0 *
         SdsModuleConfigurations.MK4_L2.getDriveReduction() *
         SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;
            // 6380.0 / 60.0 *
                // SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);
    }
    public static class AutoConstants {
        public static double MIN_SPACE_BETWEEN_POINTS = 0.5;
//        public static final double[] FRONT_LEFT = {0.5, 0.5};
//        public static final double[] FRONT_RIGHT = {0.5,-0.5};
//        public static final double[] BACK_LEFT = {-0.5,0.5};
//        public static final double[] BACK_RIGHT = {-0.5,-0.5};

        public static double MAX_SPEED_CONTROLLER_METERS_PER_SECOND = 2;
        public static double MAX_ACCELERATION_CONTROLLER_METERS_PER_SECOND_SQUARED = 2;
        public static TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(2.5 * Math.PI, 1.5 * Math.PI);

        public static double P_X_CONTROLLER = 2.2;
        public static double I_X_CONTROLLER = 0.025;
        public static double D_X_CONTROLLER = 0;

        public static double P_Y_CONTROLLER = 2.2;
        public static double I_Y_CONTROLLER = 0.025;
        public static double D_Y_CONTROLLER = 0;

        public static double TRANSLATION_FF = 0.3;

        public static double P_THETA_CONTROLLER = 2.2;
        public static double I_THETA_CONTROLLER = 0;
        public static double D_THETA_CONTROLLER = 0;
        public static double THETA_FF = 7.5;
    }

    public static class IDConstants {
        public static final int[] TALON_FX_IDS = new int[]{5, 6, 8, 9, 11, 12, 14, 15, 22};
        public static final int[] SPARK_MAX_IDS = new int[]{};


        public static final int HANGER_TALON_FX_MOTOR_ID = 22;
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

    public static class HangerConstants {
        public static final int HANGER_TALON_FX_MOTOR_ID = 22;
        public static final double EXTEND_DISTANCE = 0.0;
        public static final double RETRACT_DISTANCE = 0.0;
        public static final double PARTIAL_DISTANCE = 0.0;
        public static final int SOLENOID_LEFT_FORWARD = 1;
        public static final int SOLENOID_LEFT_BACKWARD = 2;
        public static final int SOLENOID_RIGHT_FORWARD = 3;
        public static final int SOLENOID_RIGHT_BACKWARD = 4;
    }
}
