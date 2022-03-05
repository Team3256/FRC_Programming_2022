package frc.robot.helper.logging;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * NOTE:
 * Do not use this class during competition
 * Only use for testing and debugging
 * Frequent Networktables posting takes long CPU time
 */

public class EmergencyLogger {
    private static NetworkTableInstance inst = NetworkTableInstance.getDefault();

    private static NetworkTable table = inst.getTable("Logs");

    /**
     * @param logName
     * @param msg
     * posts message on specified log
     * do not use during competition
     */
    public static void log(String logName, String msg){
        NetworkTableEntry entry = table.getEntry(logName);
        entry.setString(entry.getString("")+msg+"\n");
    }

    /**
     * @param logName
     * clears specified log of messages
     * do not use during competition
     */
    public static void clear(String logName){
        NetworkTableEntry entry = table.getEntry(logName);
        entry.setString("");
    }
}
