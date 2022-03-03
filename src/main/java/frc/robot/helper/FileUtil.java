package frc.robot.helper;

import edu.wpi.first.wpilibj.Filesystem;
import frc.robot.helper.logging.RobotLogger;

import java.io.*;
import java.nio.file.Paths;
import java.util.logging.Level;
import java.util.logging.Logger;

import static frc.robot.Constants.LimelightAutoCorrectConstants.POLYNOMIAL_FILENAME;

public class FileUtil {
    private static final RobotLogger logger = new RobotLogger(Limelight.class.getCanonicalName());

    public static void writeObjectToFile(String fileName, Object obj){
        String filePath = getFilePath(fileName);
        createFileIfDoesNotExist(filePath);
        try {
            ObjectOutputStream out = new ObjectOutputStream(new FileOutputStream(filePath));
            out.writeObject(obj);
            out.close();
        } catch (IOException e){
            logger.warning(e.getMessage(), e);
        }
    }
    public static Object readObjectFromFile(String fileName){
        String filePath = getFilePath(fileName);
        createFileIfDoesNotExist(filePath);
        Object ret = null;
        try {
            ObjectInputStream in = new ObjectInputStream(new FileInputStream(filePath));
            ret = in.readObject();
            in.close();
        } catch (Exception e){
            logger.warning(e.getMessage(), e);
        }
        return ret;
    }
    public static void createFileIfDoesNotExist(String fileName){
        String filePath = getFilePath(fileName);
        File file = new File(filePath);
        if (!file.isFile()) {
            try {
                file.createNewFile();
            } catch (IOException e) {
                logger.warning(e.getMessage(), e);
            }
        }
    }
    public static String getFilePath(String fileName){
        return Paths.get(Filesystem.getDeployDirectory().toString(),fileName).toString();
    }
    public static boolean fileExists(String fileName){
        String filePath = getFilePath(fileName);
        File file = new File(filePath);
        return file.isFile();
    }
    public static void deleteFile(String fileName){
        String filePath = getFilePath(fileName);
        File file = new File(filePath);
        file.delete();
    }
}
