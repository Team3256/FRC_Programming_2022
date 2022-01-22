package frc.robot.helper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LimelightConstants.*;

public class Limelight {
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
        return limelight.getEntry(value);
    }
    public static double getTx(){
        return getLimelightValue("tx").getDouble(0);
    }
    public static double getTy(){
        return getLimelightValue("ty").getDouble(0);
    }
    public static double getTa(){
        return getLimelightValue("ta").getDouble(0);
    }
    public static double getTs(){
        return getLimelightValue("ts").getDouble(0);
    }
    public static double[] getTcornx(){
        return getLimelightValue("tcornx").getDoubleArray(new double[4]);
    }
    public static double[] getTcorny(){
        return getLimelightValue("tcorny").getDoubleArray(new double[4]);
    }
    public static double getDistanceToTarget(){
        return (TARGET_HEIGHT_INCHES-MOUNTING_HEIGHT_INCHES)/Math.tan(toRadians(MOUNTING_ANGLE_DEG+getTy()));
    }
    public static double toRadians(double degrees){
        return degrees * Math.PI/180.0;
    }
    public static double getTuned(double x){
        return 1.04*x + -9.54;
    }
}
