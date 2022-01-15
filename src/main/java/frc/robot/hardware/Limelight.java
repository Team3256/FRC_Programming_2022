package frc.robot.hardware;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

public class Limelight {
    private NetworkTableInstance inst;
    private NetworkTableEntry limeLightTx, limeLightTy, limeLightTa, limeLightTs, limeLightTcornx, limeLightTcorny;
    private double tx, ty, ta, ts;
    private double [] tcornx, tcorny;

    public void init() {
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
}
