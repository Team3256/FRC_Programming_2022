package frc.robot.hardware;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import frc.team3256.warriorlib.loop.Loop;

public class Limelight implements Loop {
    private NetworkTableInstance inst;
    private NetworkTableEntry limeLightTx, limeLightTy, limeLightTa, limeLightTs, limeLightTcornx, limeLightTcorny;
    private double tx, ty, ta, ts;
    private double [] tcornx, tcorny;

    //singleton
    private static Limelight instance;
    public static Limelight getInstance() { return instance == null ? instance = new Limelight() : instance; }

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
    @Override
    public void update(double timestamp) {
        tx = limeLightTx.getDouble(0);
        ty = limeLightTy.getDouble(0);
        ta = limeLightTa.getDouble(0);
        ts = limeLightTs.getDouble(0);
        tcornx = limeLightTcornx.getDoubleArray(new double[4]);
        tcorny = limeLightTcorny.getDoubleArray(new double[4]);
    }

    @Override
    public void init(double timestamp) {

    }
    @Override
    public void end(double timestamp) {

    }
}
