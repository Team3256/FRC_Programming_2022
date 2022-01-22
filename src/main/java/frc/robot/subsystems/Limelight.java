package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LimelightConstants.*;

public class Limelight extends SubsystemBase {
    private NetworkTableInstance inst;
    private NetworkTableEntry limeLightTx, limeLightTy, limeLightTa, limeLightTs, limeLightTcornx, limeLightTcorny;
    private double tx, ty, ta, ts;
    private double [] tcornx, tcorny;

    public Limelight() {
        //Setting up NetworkTables
        inst = NetworkTableInstance.getDefault();
        NetworkTable limelight = inst.getTable("limelight");

        //Getting values
        limeLightTx = limelight.getEntry("tx");
        limeLightTy = limelight.getEntry("ty");
        limeLightTa = limelight.getEntry("ta");
        limeLightTs = limelight.getEntry("ts");
        limeLightTcornx = limelight.getEntry("tcornx");
        limeLightTcorny = limelight.getEntry("tcorny");

        //Setting up default stream
        limelight.getEntry("ledMode").setNumber(0); //Uses LED Mode in current pipeline
        limelight.getEntry("camMode").setNumber(0); //Uses Vision Processor Mode
        limelight.getEntry("pipeline").setNumber(0); //Uses pipeline #0
        limelight.getEntry("stream").setNumber(2); //Driver Camera Main, Vision Camera Lower-Right Corner
        limelight.getEntry("snapshot").setNumber(0); //Takes no snapshots
    }
    @Override
    public void periodic() {
        tx = limeLightTx.getDouble(0);
        ty = limeLightTy.getDouble(0);
        ta = limeLightTa.getDouble(0);
        ts = limeLightTs.getDouble(0);
        tcornx = limeLightTcornx.getDoubleArray(new double[4]);
        tcorny = limeLightTcorny.getDoubleArray(new double[4]);
    }
    public double getDistanceToTarget(){
        return (TARGET_HEIGHT_INCHES-MOUNTING_HEIGHT_INCHES)/Math.tan(toRadians(MOUNTING_ANGLE_DEG+ty));
    }
    public double toRadians(double degrees){
        return degrees * Math.PI/180.0;
    }

    public double getTuned(double x){
        return 1.04*x + -9.54;
    }
}
