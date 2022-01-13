package frc.robot.helper.logging;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;

import java.io.IOException;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.logging.FileHandler;
import java.util.logging.Logger;

import static frc.robot.Constants.LoggingConstants.*;

public class RobotLogger {

    //Does not Allow Instances
    private RobotLogger(){}
    static private Logger globalLogger;

    /**
     * Sets Up Logging Format and Output
     */
    static public void setup() {

        String homePath = Filesystem.getOperatingDirectory().getAbsolutePath();

        // get the global logger to configure it, applies to all loggers
        globalLogger = Logger.getLogger("");

        globalLogger.setLevel(LOG_LEVEL);
        globalLogger.getHandlers()[0].setLevel(CONSOLE_LEVEL);

        //If Running Unit Tests / Simulation
        if (!RobotBase.isReal()) {
            return;
        }

        if (Paths.get("u").toFile().exists()) {
            normalLog("u"); // USB Defaults to /u for Mounting
        } else {
            System.err.println("NO USB DRIVE");

            if (FORCE_NORMAL_INTERNAL) {
                System.err.println("Forcing Normal Logging to Internal");
                normalLog(homePath);
            } else {
                emergencyLog();
            }
        }



    }

    /**
     * Logs to USB Drive with HTML + Text
     */
    static private void normalLog(String pathToLogInto){
        Path txtFilePath = Paths.get(pathToLogInto, TXT_FILE_NAME);
        Path htmlFilePath = Paths.get(pathToLogInto, HTML_FILE_NAME);

        try {
            FileHandler fileTxtHandler = new FileHandler(
                    txtFilePath.toString(), TXT_LOG_MAX_SIZE, TXT_LOG_MAX_FILES, false);
            FileHandler fileHTMLHandler = new FileHandler(
                    htmlFilePath.toString(), HTML_LOG_MAX_SIZE, HTML_LOG_MAX_FILES, false);


            fileTxtHandler.setFormatter(new OneLineFormatter());
            globalLogger.addHandler(fileTxtHandler);

            fileHTMLHandler.setFormatter(new HtmlFormatter());
            globalLogger.addHandler(fileHTMLHandler);

        } catch(IOException e){
            System.err.println("Normal Log FAILED - IOException - Going to Emergency Log");
            e.printStackTrace();
            emergencyLog();
        }
    }

    /**
     * When normal Logging Fails, Log minimally to Internal Storage
     */
    static private void emergencyLog() {
        for (int i = 0; i < 3; i++)
            System.err.println("Emergency LOG - Logging to Internal");

        try {
            String homePath = Filesystem.getOperatingDirectory().getAbsolutePath();

            Path emergencyTxtPath = Paths.get(homePath, TXT_FILE_NAME);
            FileHandler fileTxtHandler = new FileHandler(
                    emergencyTxtPath.toString(), EMERGENCY_TXT_MAX_SIZE, EMERGENCY_TXT_MAX_FILES, false);

            fileTxtHandler.setFormatter(new OneLineFormatter());
            globalLogger.addHandler(fileTxtHandler);

        } catch (IOException e){
            System.err.println("EMERGENCY LOG FAILED - CANT CREATE FILE");
            e.printStackTrace();
            System.err.println("LOGGING ONLY TO CONSOLE");
        }
    }

}