// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.hardware.TalonConfiguration;

import java.util.logging.Level;

public final class Constants {

    public static final boolean DEBUG = false;

    public static class LimelightAutoCorrectConstants {
        public static final int PACE_SIZE = 5;
        public static final int PACES = 40;
        public static final String POLYNOMIAL_FILENAME = "Polynomial.txt";
        public static final int POLYNOMIAL_DEGREE = 5;
    }
    public static class LimelightConstants {
        public static final double MOUNTING_HEIGHT_INCHES = 24.5;
        public static final double TARGET_HEIGHT_INCHES = 98;
        public static final double MOUNTING_ANGLE_DEG = 43;
    }
    public static class TurretConstants {
        public static final double kP = 0;
        public static final double kI = 0;
        public static final double kD = 0;
        public static final double DEFAULT_TURRET_SPEED = 50;
        public static final double TURRET_TOLERANCE_TX = 0.5;
        public static final double GEAR_RATIO = 0.25;
    }
    public static class FeederConstants {
        public static final double DEFAULT_FEEDER_SPEED = 50;
        public static final int MAX_BALL_COUNT = 2; //change later
    }
    public static class SwerveConstants {
        public static final double DRIVETRAIN_MOTOR_DEADZONE_VOLTS = 0.55;
        public static final double DRIVETRAIN_TRACK_METERS = 0.4445;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(168.8379); //357
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(233.1738); //179
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = -Math.toRadians(349.8926);
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = -Math.toRadians(52.8223); //179

        public static final double MAX_METERS_PER_SECOND = 10;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                 SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                 SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        private static final double ANGULAR_VELOCITY_CONSTANT = 1;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = ANGULAR_VELOCITY_CONSTANT * MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        //Swerve Turret

        public static final boolean SWERVE_TURRET_TUNING = false;


        //Non-final Allow for Changing via Smart Dashboard
        public static double SWERVE_TURRET_KP = 0.1;
        public static double SWERVE_TURRET_KI = 0;
        public static double SWERVE_TURRET_KD = 0;
        public static boolean IS_TUNING_SWERVE_TURRET = true;

        public static double SWERVE_TURRET_STATIONARY_MIN = 0.4;

        public static final double TURN_TOLERANCE = 0;
        public static final double TURN_RATE_TOLERANCE = 0;
    }
    public static class AutoConstants {
        public static double MIN_SPACE_BETWEEN_POINTS = 0.5;

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

        public static double P_THETA_CONTROLLER = 1.8;
        public static double I_THETA_CONTROLLER = 0.01;
        public static double D_THETA_CONTROLLER = 0;
    }

    public static class IDConstants {

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

        public static final int[] TALON_FX_IDS = new int[]{5, 6, 8, 9, 11, 12, 14, 15};
      
        public static final int[] SPARK_MAX_IDS = new int[]{};

        public static final int PID_SHOOTER_MOTOR_ID_LEFT = 7;
        public static final int PID_SHOOTER_MOTOR_ID_RIGHT = 8;


        public static final int TURRET_ID = 34;
        public static final int FEEDER_MOTOR_ID = 35;

        public static final int HOOD_MOTOR_ID = 0;

        public static final int HANGER_MASTER_TALON_ID = 36;
        public static final int HANGER_FOLLOWER_TALON_ID = 37;

        //Pneumatic IDs
        public static final int HANGER_SOLENOID_LEFT_FORWARD = 1;
        public static final int HANGER_SOLENOID_LEFT_BACKWARD = 2;
        public static final int HANGER_SOLENOID_RIGHT_FORWARD = 3;
        public static final int HANGER_SOLENOID_RIGHT_BACKWARD = 4;
        public static final int HANGER_SOLENOID_LEFT_AIRBRAKE_FORWARD = 5;
        public static final int HANGER_SOLENOID_LEFT_AIRBRAKE_BACKWARD = 6;
        public static final int HANGER_SOLENOID_RIGHT_AIRBRAKE_FORWARD = 5;
        public static final int HANGER_SOLENOID_RIGHT_AIRBRAKE_BACKWARD = 6;

        // DIO Channels
        public static final int HANGER_LIMITSWITCH_CHANNEL = 0;
        public static final int HOOD_LIMITSWITCH_CHANNEL = 1;

        public static final int IR_TRANSFER_BEGINNING_CHANNEL = 2; //change later
        public static final int IR_TRANSFER_MIDDLE_CHANNEL = 3; //change later
        public static final int IR_TRANSFER_END_CHANNEL = 4; //change later

        // I2C
        public static final byte I2C_MUX_ADDRESS = 0x70;
        public static final int I2C_COLOR_SENSOR_FIXED_ADDRESS = 0x52;


        public static final byte BALL_COLOR_SENSOR_MUX_PORT = 0;
        public static final byte LEFT_ALIGN_COLOR_SENSOR_MUX_PORT = 1;
        public static final byte RIGHT_ALIGN_COLOR_SENSOR_MUX_PORT = 2;


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
        public static final int TXT_LOG_MAX_FILES = 40;
        public static final int HTML_LOG_MAX_FILES = 20;

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

        public static final double HANGER_MASTER_TALON_PID_P = 0;
        public static final double HANGER_MASTER_TALON_PID_I = 0;
        public static final double HANGER_MASTER_TALON_PID_D = 0;
        public static final double HANGER_MASTER_TALON_PID_F = 0;

        public static final InvertType INVERT_TYPE = InvertType.None;

        public static TalonConfiguration.TalonFXPIDFConfig PIDF_CONSTANTS = new TalonConfiguration.TalonFXPIDFConfig(
                HANGER_MASTER_TALON_PID_P,
                HANGER_MASTER_TALON_PID_I,
                HANGER_MASTER_TALON_PID_D,
                HANGER_MASTER_TALON_PID_F
        );

        public static TalonConfiguration MASTER_CONFIG = new TalonConfiguration(PIDF_CONSTANTS, INVERT_TYPE, NeutralMode.Brake);
        public static TalonConfiguration FOLLOWER_CONFIG = TalonConfiguration.createFollowerConfig(MASTER_CONFIG, InvertType.OpposeMaster);

        public static final double GEAR_RATIO = 0; // from spool to motor

        public static final double EXTEND_DISTANCE = 0.0; // in Rotations of Spool
        public static final double PARTIAL_DISTANCE = 0.0; // in Rotations of Spool

        public static final double RETRACT_PERCENT_SPEED = 0.0;

        public static final double PNEUMATIC_WAIT_DURATION = 0; //in Seconds
        public static final double EXTEND_WAIT = 0; //in Seconds
        public static final double RETRACT_WAIT = 0.0; //in Seconds
        public static final double PARTIAL_EXTEND_WAIT = 0; //in Seconds

        public static final Color TAPE_COLOR = new Color(0.251413600330047,0.476727327560996,0.272224140677223);
        public static final double MAX_CONFIDENCE_DEVIATION = 0.01;
        public static final int HANGER_ALIGN_ROTATION_VOLTAGE = 2;
        public static final double HANGER_ALIGN_METERS_PER_SECOND = 0.1;
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
        public static final double ENTRY_ANGLE_INTO_HUB = 50.0; // TODO: From tuning, find entry angle

        //PID
        public static final double SHOOTER_MASTER_TALON_PID_P = 0;
        public static final double SHOOTER_MASTER_TALON_PID_I = 0;
        public static final double SHOOTER_MASTER_TALON_PID_D = 0;
        public static final double SHOOTER_MASTER_TALON_PID_F = 0;

        public static final String VEL_CALIB_FILENAME = ""; // TODO: Add filename for the .csv file with training data points
        public static final String HOOD_CALIB_FILENAME = ""; // TODO: Add filename for the .csv file with training data points

        // Hood Angle Constants
        public static final double HOOD_SLOW_REVERSE_PERCENT = -0.05;
        // In sensor units
        public static final double HOOD_ANGLE_UPPER_LIMIT = 2048 * 15; // TODO: Change to actual amount from 15 rotations
        public static final double HOOD_ANGLE_LOWER_LIMIT = 0;
    }
}
