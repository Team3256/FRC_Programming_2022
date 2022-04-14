// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import frc.robot.helper.LED.helpers.*;
import frc.robot.helper.LED.PatternGenerators.*;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.hardware.TalonConfiguration;

import java.util.Arrays;
import java.util.HashMap;
import java.util.LinkedHashMap;

import frc.robot.helper.Polynomial;

import frc.robot.helper.shooter.ShooterPreset;
import frc.robot.helper.shooter.TrainingDataPoint;
import frc.robot.subsystems.ShooterSubsystem.ShooterLocationPreset;

import java.util.List;

import java.util.logging.Level;

import static frc.robot.Constants.LEDConstants.LEDSectionName.*;
import static java.util.Map.entry;

public final class Constants {

    public static final boolean DEBUG = true;
    public static final boolean LOG_DEBUG_TO_CONSOLE = false;  // Requires DEBUG to be true

    public static final double PDH_FAULT_WATCHER_INTERVAL = 1;

    public static class SubsystemEnableFlags {
        public static final boolean LIMELIGHT = true;

        public static final boolean SHOOTER = true;
        public static final boolean TRANSFER = true;
        public static final boolean INTAKE = true;

        public static final boolean HANGER = false;

        public static final boolean DRIVETRAIN = true;

        public static final boolean BALL_COLOR_SENSOR = false;
        public static final boolean BOTTOM_COLOR_SENSORS = false;

        public static final boolean IR_SENSORS = true;
    }

    public static class FieldConstants {
        public static final Translation2d HUB_POSITION = new Translation2d(Units.inchesToMeters(324), Units.inchesToMeters(163.99)); // in meters
        // position of the hub on the field with the origin at the blue alliance terminal (similar to path planner)
    }

    public static class LimelightAutoCorrectConstants {
        public static final double MAX_VISION_LOCALIZATION_TRANSLATION_CORRECTION = 0.5; // in meters
        public static final double MAX_VISION_LOCALIZATION_HEADING_CORRECTION = 5; // in degrees
        public static final int PACE_SIZE = 5;
        public static final int PACES = 40;
        public static final int POLYNOMIAL_DEGREE = 5;
        public static final Polynomial LIMELIGHT_DISTANCE_TUNER = new Polynomial(new double[]{0,1,0,0,0}); // TODO: Put actual polynomial coefficients here
    }

    public static class LimelightConstants {
        public static final double MOUNTING_HEIGHT_INCHES = 24.5;
        public static final double TARGET_HEIGHT_INCHES = 98;
        public static final double MOUNTING_ANGLE_DEG = 43;
    }

    public static class TransferConstants {
        public static final double DEFAULT_TRANSFER_SPEED = 0.35; // In Percent 0.0 - 1.0
        public static final double SHOOT_FORWARD_TRANSFER_SPEED = 0.5;
        public static final double MANUAL_REVERSE_TRANSFER_SPEED = -0.35; // In Percent -1.0 - 0.0

        public static final int MAX_BALL_COUNT = 2;
        public static final int STARTING_BALL_COUNT = 1;

        public static final int MIN_BALL_COLOR_PROXIMITY = 1500; // Raw Proximity value 0 - 2047 (0 being far away)

        public static final Color RED_BALL_COLOR = new Color(1, 0, 0); // TODO: Set to Measured Ball Color

        public static final Color BLUE_BALL_COLOR = new Color(0, 0, 1); // TODO: Set to Measured Ball Color

        public static final double MAX_BALL_COLOR_DEVIATION = 0.01;

    }

    public static class SwerveConstants {
        public static final boolean INVERT_TURN = true;
        public static final double DRIVETRAIN_MOTOR_DEADZONE_VOLTS = 0.4;
        public static final double DRIVETRAIN_TRACK_METERS = 0.4445;
        public static final double DRIVETRAIN_WHEELBASE_METERS = 0.4445;

        public static final double X_ACCEL_RATE_LIMIT = 10;
        public static final double X_DECEL_RATE_LIMIT = 10;
        public static final double Y_ACCEL_RATE_LIMIT = 10;
        public static final double Y_DECEL_RATE_LIMIT = 10;

        public static final double GYRO_YAW_OFFSET = 45; // degrees //TODO: CHECK OFFSET is right, Intake is forward

        /* Enabled | Limit(amp) | Trigger Threshold(amp) | Trigger Threshold Time(s)  */
        public static final StatorCurrentLimitConfiguration STATOR_CURRENT_LIMIT = new StatorCurrentLimitConfiguration(true, 80,80,0); // in Amps, limits amount of current drawn to brake


        // All in Degrees
        public static final double FRONT_LEFT_MODULE_STEER_OFFSET = 2.977361;
        public static final double FRONT_RIGHT_MODULE_STEER_OFFSET = 0.846566;
        public static final double BACK_LEFT_MODULE_STEER_OFFSET = 3.166089;
        public static final double BACK_RIGHT_MODULE_STEER_OFFSET = 4.067925;


        public static final double MAX_METERS_PER_SECOND = 10;

        public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
                SdsModuleConfigurations.MK4_L2.getDriveReduction() *
                SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI;

        private static final double ANGULAR_VELOCITY_CONSTANT = 1;
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = ANGULAR_VELOCITY_CONSTANT * MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVETRAIN_TRACK_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0);

        //Swerve Turret

        public static final boolean SWERVE_TURRET_TUNING = true;


        //Non-final Allow for Changing via Smart Dashboard
        public static double SWERVE_TURRET_KP = 0.05;
        public static double SWERVE_TURRET_KI = 0.001;
        public static double SWERVE_TURRET_KD = 0.001;

        public static double SWERVE_ODOMETRY_TURRET_KP = 0.1;
        public static double SWERVE_ODOMETRY_TURRET_KI = 0;
        public static double SWERVE_ODOMETRY_TURRET_KD = 0.007;

        public static double SWERVE_TURRET_STATIONARY_KP = 0.1;
        public static double SWERVE_TURRET_STATIONARY_KI = 0.2;
        public static double SWERVE_TURRET_STATIONARY_KD = 0;

        public static double SWERVE_TURRET_STATIONARY_MIN = 0.4;

        public static double SWERVE_TURRET_OPERATOR_DEADZONE = 0.4;
        public static double SWERVE_TURRET_OPERATOR_INFLUENCE = 1;

        public static final double TURN_TOLERANCE = 0.1;
        public static final double TURN_RATE_TOLERANCE = 1;

        // TELEOP
        public static final double AUTO_AIM_BREAKOUT_TOLERANCE = 0.05;

    }
    public static class AutoConstants {
        public static final boolean AUTO_DEBUG = false;
        public static final double COMMAND_MARKER_THRESHOLD = 0.05; // meters

        public static double MAX_SPEED_CONTROLLER_METERS_PER_SECOND = 13;
        public static double MAX_ACCELERATION_CONTROLLER_METERS_PER_SECOND_SQUARED = 8;
        public static TrapezoidProfile.Constraints THETA_CONTROLLER_CONSTRAINTS = new TrapezoidProfile.Constraints(2.5 * Math.PI, 1.5 * Math.PI);

        public static double P_X_CONTROLLER = 2.2;
        public static double I_X_CONTROLLER = 0.025;
        public static double D_X_CONTROLLER = 0;

        public static double P_Y_CONTROLLER = 2.2;
        public static double I_Y_CONTROLLER = 0.025;
        public static double D_Y_CONTROLLER = 0.05;

        public static double TRANSLATION_FF = 0.3;

        public static double P_THETA_CONTROLLER = 7.5;
        public static double I_THETA_CONTROLLER = 0.02;
        public static double D_THETA_CONTROLLER = 1.2;
    }

    public static class IDConstants {
        public static final int[] TALON_FX_IDS = new int[]{2,4,5,7,8,10,11,13,14,16};
        public static final int[] SPARK_MAX_IDS = new int[]{};

        public static final String ROBORIO_CAN_BUS = "rio";

        public static final int PNEUMATICS_HUB_ID = 17;

        public static final int PDH_ID = 0;

        public static final int BACK_LEFT_MODULE_DRIVE_MOTOR_ID = 16;
        public static final int BACK_LEFT_MODULE_STEER_ENCODER_ID = 15;
        public static final int BACK_LEFT_MODULE_STEER_MOTOR_ID = 14;

        public static final int FRONT_LEFT_MODULE_DRIVE_MOTOR_ID = 13;
        public static final int FRONT_LEFT_MODULE_STEER_ENCODER_ID = 12;
        public static final int FRONT_LEFT_MODULE_STEER_MOTOR_ID = 11;

        public static final int DRIVETRAIN_PIGEON_ID = 9;

        public static final int FRONT_RIGHT_MODULE_DRIVE_MOTOR_ID = 7;
        public static final int FRONT_RIGHT_MODULE_STEER_ENCODER_ID = 6;
        public static final int FRONT_RIGHT_MODULE_STEER_MOTOR_ID = 5;

        public static final int BACK_RIGHT_MODULE_DRIVE_MOTOR_ID = 4;
        public static final int BACK_RIGHT_MODULE_STEER_ENCODER_ID = 3;
        public static final int BACK_RIGHT_MODULE_STEER_MOTOR_ID = 2;

        // Power Distribution Hub = 1 (Required + Hardcoded)

        public static final String MANI_CAN_BUS = "mani";


        public static final int HANGER_RIGHT_FOLLOWER_TALON_ID = 7;

        public static final int HANGER_LEFT_MASTER_TALON_ID = 6;

        public static final int INTAKE_MOTOR_ID = 5;

        public static final int TRANSFER_MOTOR_ID = 4;

        public static final int HOOD_MOTOR_ID = 3;

        public static final int PID_SHOOTER_MOTOR_ID_RIGHT = 2;
        public static final int PID_SHOOTER_MOTOR_ID_LEFT = 1;

        //Pneumatic IDs
        public static final int LEFT_HANGER_SOLENOID_FORWARD = 15;
        public static final int LEFT_HANGER_SOLENOID_BACKWARD = 8;

        public static final int RIGHT_HANGER_SOLENOID_FORWARD = 14;
        public static final int RIGHT_HANGER_SOLENOID_BACKWARD = 9;

        public static final int INTAKE_SOLENOID_LEFT_BACKWARD = 11; //backward is up
        public static final int INTAKE_SOLENOID_RIGHT_BACKWARD = 10;

        public static final int INTAKE_SOLENOID_LEFT_FORWARD = 12; // forward is down
        public static final int INTAKE_SOLENOID_RIGHT_FORWARD = 13;


        // DIO Channels
        public static final int HANGER_LIMITSWITCH_CHANNEL = 5;
        public static final int HOOD_LIMITSWITCH_CHANNEL = 0;

        public static final int IR_TRANSFER_BEGINNING_CHANNEL = 9;
        public static final int IR_TRANSFER_MIDDLE_CHANNEL = 7;
        public static final int IR_TRANSFER_END_CHANNEL = 2;

        // I2C
        public static final byte I2C_MUX_ADDRESS = 0x70;
        public static final int I2C_COLOR_SENSOR_FIXED_ADDRESS = 0x52;


        public static final byte BALL_COLOR_SENSOR_MUX_PORT = 6;
        public static final byte RIGHT_ALIGN_COLOR_SENSOR_MUX_PORT = 3;
        public static final byte LEFT_ALIGN_COLOR_SENSOR_MUX_PORT = 5;

        // PWM
        public static final int LED_STRIP_PWM_PORT = 0;


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

        public static final double HANGER_MASTER_TALON_PID_P = 0.1;
        public static final double HANGER_MASTER_TALON_PID_I = 0;
        public static final double HANGER_MASTER_TALON_PID_D = 0;
        public static final double HANGER_MASTER_TALON_PID_F = 0;

        public static final InvertType INVERT_TYPE = InvertType.FollowMaster;

        public static TalonConfiguration.TalonFXPIDFConfig PIDF_CONSTANTS = new TalonConfiguration.TalonFXPIDFConfig(
                HANGER_MASTER_TALON_PID_P,
                HANGER_MASTER_TALON_PID_I,
                HANGER_MASTER_TALON_PID_D,
                HANGER_MASTER_TALON_PID_F
        );

        public static TalonConfiguration MASTER_CONFIG = new TalonConfiguration(PIDF_CONSTANTS, INVERT_TYPE, NeutralMode.Brake);
        public static TalonConfiguration FOLLOWER_CONFIG = TalonConfiguration.createFollowerConfig(MASTER_CONFIG, InvertType.OpposeMaster);

        public static final double GEAR_RATIO = 0; // from spool to motor

        public static final double EXTEND_DISTANCE = 126391; // in Sensor units
        public static final double PARTIAL_DISTANCE = 20000.0; // Sensor Units
        public static final double ADJUSTMENT_RETRACT_DISTANCE = 1000.0; //in Rotations of Spool

        public static final double HANGER_ZEROING_PERCENT_SPEED = 0.25;

        public static final double HANGER_RETRACT_PERCENT_SPEED = 0.6;

        public static final double CURRENT_THRESHOLD = 16.0; //in Amps

        public static final double PNEUMATIC_WAIT_DURATION = 0; //in Seconds
        public static final double EXTEND_WAIT = 0; //in Seconds
        public static final double RETRACT_WAIT = 0.0; //in Seconds
        public static final double PARTIAL_EXTEND_WAIT = 0; //in Seconds

        public static final Color TAPE_COLOR = new Color(0.251413600330047,0.476727327560996,0.272224140677223);
        public static final double MAX_TAPE_COLOR_CONFIDENCE_DEVIATION = 0.01;
        public static final int HANGER_ALIGN_ROTATION_VOLTAGE = 2;
        public static final double HANGER_ALIGN_METERS_PER_SECOND = 0.1;
    }

    public static class IntakeConstants {
        public static final double INTAKE_FORWARD_SPEED = 1; // In Percent 0.0 - 1.0
        public static final double INTAKE_BACKWARD_SPEED = -0.5; // In Percent -1.0 - 0.0
    }

    public static class ShooterConstants {
        // Constant Shooting Section
        public static final double RADIUS_UPPER_HUB = 0.61; // in m
        public static final double SHOOTER_HEIGHT = 0.51; // in m
        public static final double UPPER_HUB_AIMING_HEIGHT = 2.725427; // in m

        // Tuning Section
        public static final double DELTA_AIM_HEIGHT_FACTOR = 0.0; // TODO: Set delta aim height factor from tuning
        public static final double DELTA_DISTANCE_TO_TARGET_FACTOR = 0.0; // TODO: Set delta distance from tuning
        public static final double SET_POINT_ERROR_MARGIN = 0.05; // in percent TODO: Set margin of error for initiation speed test
        public static final double OFFSET_HEIGHT_FACTOR = 0.0; // TODO: From tuning, set offset height
        public static final double OFFSET_DISTANCE_FACTOR = 0.0; // TODO: From tuning, set offset distance
        public static final double ENTRY_ANGLE_INTO_HUB = 50.0; // TODO: From tuning, find entry angle

        //PID
        public static final double SHOOTER_MASTER_TALON_PID_P = 0.0005;
        public static final double SHOOTER_MASTER_TALON_PID_I = 0;
        public static final double SHOOTER_MASTER_TALON_PID_D = 0.000008;
        public static final double SHOOTER_MASTER_TALON_PID_F = 0;

        // Hood Angle Constants
        public static final double HOOD_SLOW_REVERSE_PERCENT = -0.50;
        // In sensor units
        public static final double HOOD_ANGLE_UPPER_LIMIT = 2048 * 15; // TODO: Change to actual amount from 15 rotations
        public static final double HOOD_ANGLE_LOWER_LIMIT = 0;

        // Presets
        public static final HashMap<ShooterLocationPreset, ShooterPreset> ALL_SHOOTER_PRESETS = HashMapFiller.populateHashMap(
                entry(ShooterLocationPreset.LAUNCHPAD, new ShooterPreset(2600, 235000, 0, "Launchpad")),
                entry(ShooterLocationPreset.TARMAC_VERTEX, new ShooterPreset(2290, 140000, 0, "Tarmac Vertex"))
        ); // TODO: Create all shooter presets

        // Velocity Training Points
        public static final List<TrainingDataPoint> ALL_SHOOTER_CALIB_TRAINING = Arrays.asList(
                new TrainingDataPoint(100, 123, 1.23, 110) // TODO: Change this to actual calibrated training (given test)
        ); // TODO: Create all training data

        public static final List<TrainingDataPoint> SIMPLE_CALIB_TRAINING = Arrays.asList(
                new TrainingDataPoint(0, 0, 0) //TODO: SET THIS
        );
    }
    public static class LEDConstants {
        public static final double MIN_WAIT_TIME_BETWEEN_INSTRUCTIONS = 0.03;  // In Seconds

        public enum LEDSectionName {
            BALL_COLOR, AUTO_AIM, DEBUG_SECTION
        }

        public static final BallColorPatternGenerator BALL_PATTERN = new BallColorPatternGenerator();
        public static final AutoAimPatternGenerator AUTO_AIM_PATTERN = new AutoAimPatternGenerator();
        public static final DebugLEDWalkUpPatternGenerator DEBUG_LED = new DebugLEDWalkUpPatternGenerator();

        // Defines order of Sections (Thus LinkedHashMap)
        public static final LinkedHashMap<LEDSectionName, LEDSectionAttributes> SECTIONS =
                HashMapFiller.populateLinkedHashMap(
                        entry(BALL_COLOR, new LEDSectionAttributes(0, 0.7, BALL_PATTERN)),
                        entry(AUTO_AIM, new LEDSectionAttributes(0.7, 1, AUTO_AIM_PATTERN))
                );

        // Defines Ranges
        public static final LEDRange[] RANGES = {
                new LEDRange(0,200, 180)
        };
    }

    public static class PatternGeneratorConstants{
        public static final LEDColor RED_BALL_LED_COLOR = LEDColor.fromRGB(10,0,0);
        public static final LEDColor BLUE_BALL_LED_COLOR = LEDColor.fromRGB(0,0,10);

        public static final LEDColor AUTO_AIM_LED_COLOR = LEDColor.fromRGB(0, 100, 0);
    }

    public static final double POKERFACE_ANGLE_MARGIN_OF_ERROR = 45;
    public static final int CYCLES_PER_CANDLE_UPDATE = 10;

}
