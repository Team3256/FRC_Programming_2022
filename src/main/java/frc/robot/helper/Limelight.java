package frc.robot.helper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Logger;

import static frc.robot.Constants.LimelightConstants.*;

public class Limelight {
    private static final Logger logger = Logger.getLogger(Limelight.class.getCanonicalName());
    private static NetworkTable limelight;
    public static void init() {
        //Setting up NetworkTables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        limelight = inst.getTable("limelight");

        //Setting up default stream
        limelight.getEntry("ledMode").setNumber(0); //Uses LED Mode in current pipeline
        limelight.getEntry("camMode").setNumber(0); //Uses Vision Processor Mode
        limelight.getEntry("pipeline").setNumber(0); //Uses pipeline #0
        limelight.getEntry("stream").setNumber(2); //Driver Camera Main, Vision Camera Lower-Right Corner
        limelight.getEntry("snapshot").setNumber(0); //Takes no snapshots
    }

    private static NetworkTableEntry getLimelightValue(String value){
        if (limelight == null) {
            logger.severe("Limelight not Initialized! Returning Bad NetworkTable!");
            return new NetworkTableEntry(NetworkTableInstance.getDefault(), 0);
        }
        return limelight.getEntry(value);
    }

    /**
     * @return Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
     */
    public static double getTx(){
        return getLimelightValue("tx").getDouble(0);
    }

    /**
     * @return Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
     */
    public static double getTy(){
        return getLimelightValue("ty").getDouble(0);
    }

    /**
     * @return Target Area (0% of image to 100% of image)
     */
    public static double getTa(){
        return getLimelightValue("ta").getDouble(0);
    }

    /**
     * @return Skew or rotation (-90 degrees to 0 degrees)
     */
    public static double getTs(){
        return getLimelightValue("ts").getDouble(0);
    }

    /**
     * @return Number array of corner coordinates [x0,x1,etc]
     */
    public static double[] getTcornx(){
        return getLimelightValue("tcornx").getDoubleArray(new double[4]);
    }

    /**
     * @return Number array of corner coordinates [y0,y1,etc]
     */
    public static double[] getTcorny(){
        return getLimelightValue("tcorny").getDoubleArray(new double[4]);
    }

    public static double getDistanceToTarget(){
        return getTuned(TARGET_HEIGHT_INCHES-MOUNTING_HEIGHT_INCHES)/Math.tan(toRadians(MOUNTING_ANGLE_DEG+getTy()));
    }
    public static double getInaccurateDistanceToTarget(){
        return (TARGET_HEIGHT_INCHES-MOUNTING_HEIGHT_INCHES)/Math.tan(toRadians(MOUNTING_ANGLE_DEG+getTy()));
    }

    public static double toRadians(double degrees){
        return degrees * Math.PI/180.0;
    }

    /**
     * @param x the not tuned distance
     * tunes the distance you put in to the actual distance
     * @return the actual distance
     */
    private static double getTuned(double x){
        return 1.04*x + -9.54;
    }
}
