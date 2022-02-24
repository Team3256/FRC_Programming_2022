package frc.robot.helper;

import edu.wpi.first.wpilibj.Filesystem;

import java.io.*;
import java.nio.file.Paths;
import java.util.logging.Level;
import java.util.logging.Logger;

import static frc.robot.Constants.LimelightAutoCorrectConstants.POLYNOMIAL_FILENAME;

public class FileUtil {
    private static final Logger logger = Logger.getLogger(Limelight.class.getCanonicalName());

    public static void writeObjectToFile(String fileName, Object obj){
        String filePath = getFilePath(fileName);
        createFileIfDoesNotExist(filePath);
        try {
            ObjectOutputStream out = new ObjectOutputStream(new FileOutputStream(filePath));
            out.writeObject(obj);
            out.close();
        } catch (IOException e){
            logger.log(Level.WARNING, e.getMessage(), e);
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
            logger.log(Level.WARNING, e.getMessage(), e);
        }
        return ret;
    }
    public static void createFileIfDoesNotExist(String fileName){
        String filePath = getFilePath(fileName);
        File polynomialFile = new File(filePath);
        if (!polynomialFile.isFile()) {
            try {
                polynomialFile.createNewFile();
            } catch (IOException e) {
                logger.log(Level.WARNING, e.getMessage(), e);
            }
        }
    }
    public static String getFilePath(String fileName){
        return Paths.get(Filesystem.getDeployDirectory().toString(),fileName).toString();
    }
}
