package frc.robot.helper;

import java.io.*;
import java.util.logging.Level;
import java.util.logging.Logger;

import static frc.robot.Constants.LimelightAutoCorrectConstants.POLYNOMIAL_FILE_PATH;

public class FileUtil {
    private static final Logger logger = Logger.getLogger(Limelight.class.getCanonicalName());

    public static void writeObjectToFile(String fileName, Object obj){
        createFileIfDoesNotExist(fileName);
        try {
            ObjectOutputStream out = new ObjectOutputStream(new FileOutputStream(POLYNOMIAL_FILE_PATH));
            out.writeObject(obj);
            out.close();
        } catch (IOException e){
            logger.log(Level.WARNING, e.getMessage(), e);
        }
    }
    public static Object readObjectFromFile(String fileName){
        createFileIfDoesNotExist(fileName);
        Object ret = null;
        try {
            ObjectInputStream in = new ObjectInputStream(new FileInputStream(POLYNOMIAL_FILE_PATH));
            ret = in.readObject();
            in.close();
        } catch (Exception e){
            logger.log(Level.WARNING, e.getMessage(), e);
        }
        return ret;
    }
    public static void createFileIfDoesNotExist(String fileName){
        File polynomialFile = new File(fileName);
        if (!polynomialFile.isFile()) {
            try {
                polynomialFile.createNewFile();
            } catch (IOException e) {
                e.printStackTrace();
            }
        }
    }
}
