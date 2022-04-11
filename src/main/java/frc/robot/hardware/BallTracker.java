package frc.robot.hardware;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

public class BallTracker {
    private static NetworkTable ballTracker;

    //Doesn't allow instancing
    private BallTracker(){}

    //Initialize when class is loaded
    static {
        init();
    }
    
    /**
     * initialize ball tracker network table
     */
    public static void init(){
        //Set up network tables
        NetworkTableInstance inst = NetworkTableInstance.getDefault();
        ballTracker = inst.getTable("ballTracker");
    }

    /**
     * @return ball horizontal pixel offset
     */
    public static double getDx() {
        return ballTracker.getEntry("dx").getDouble(0);
    }

    /**
     * @return image height minus ball radius pixel
     */
    public static double getDy(){
        return ballTracker.getEntry("dy").getDouble(0);
    }
}
