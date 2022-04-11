package frc.robot.hardware;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BallTracker {
    private static NetworkTable ballTracker;

    //Doesn't allow instancing
    private BallTracker(){}

    public static void init(){
        //Set up network tables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        ballTracker = inst.getTable("ballTracker");
    }
    public static double getDx() {
        return kx * ballTracker.getEntry("dx").getDouble(0);
    }
    public static double getDy(){
        return ky * ballTracker.getEntry("dy").getDouble(0);
    }
    static double kx = 10;
    static double ky = 10;
}
