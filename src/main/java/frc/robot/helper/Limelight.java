package frc.robot.helper;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.*;

import static frc.robot.Constants.LimelightAutoCorrectConstants.POLYNOMIAL_FILE_PATH;
import static frc.robot.Constants.LimelightConstants.*;

public class Limelight {
    private static NetworkTable limelight;
    public static File polynomialFile = new File(POLYNOMIAL_FILE_PATH);
    private static Polynomial corrector;

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

        //if file does not exist
        if (!polynomialFile.isFile()) {
            try {
                polynomialFile.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
        //Get polynomial from file
        ObjectInputStream in;
        try {
            in = new ObjectInputStream(new FileInputStream(POLYNOMIAL_FILE_PATH));
            corrector = (Polynomial) in.readObject();
            in.close();
        } catch (Exception e){
            //if polynomial not parsable from file, get default polynomial
            corrector = new Polynomial(new double[]{0,1});
        }
    }

    private static NetworkTableEntry getLimelightValue(String value){
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
        return (TARGET_HEIGHT_INCHES-MOUNTING_HEIGHT_INCHES)/Math.tan(toRadians(MOUNTING_ANGLE_DEG+getTy()));
    }
    public static double getTunedDistanceToTarget(){
        return corrector.getOutput(getDistanceToTarget());
    }


    public static double toRadians(double degrees){
        return degrees * Math.PI/180.0;
    }

    public static void writePolynomial(Polynomial toWrite) throws IOException {
        // Saving of object in a file
        ObjectOutputStream out = new ObjectOutputStream(new FileOutputStream(POLYNOMIAL_FILE_PATH));
        out.writeObject(toWrite);
        out.close();
    }
    /**
     * @param x the not tuned distance
     * tunes the distance you put in to the actual distance
     * @return the actual distance
     */
}
