package frc.robot.helper.CSVShooting;

import frc.robot.subsystems.FlywheelSubsystem;

import java.io.BufferedReader;
import java.io.IOException;
import java.nio.charset.StandardCharsets;
import java.nio.file.Files;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.List;
import java.util.ArrayList;
import java.util.logging.Level;
import java.util.logging.Logger;

public class ReadTrainingFromCSV {
    private static final Logger logger = Logger.getLogger(FlywheelSubsystem.class.getCanonicalName());

    private static String filename;
    private static List<TrainingDataPoint> trainingData;

    public static List<TrainingDataPoint> readDataFromCSV(String fname) {
        filename = fname;
        trainingData = new ArrayList<TrainingDataPoint>();

        Path pathToFile = Paths.get(filename);

        try (BufferedReader br = Files.newBufferedReader(pathToFile, StandardCharsets.US_ASCII)) {
            String line = br.readLine();

            String[] attributes;
            while (line != null) {
                attributes = line.split(";");

                trainingData.add(createDataPoint(attributes));

                line = br.readLine();
            }

        } catch (IOException ioe) {
            ioe.printStackTrace();
            logger.log(Level.WARNING, ioe.getMessage(), ioe);
        }

        return trainingData;
    }

    private static TrainingDataPoint createDataPoint(String[] attributes) {
        TrainingDataPoint dataPoint = new TrainingDataPoint(Double.valueOf(attributes[0]),
                Double.valueOf(attributes[0]), Double.valueOf(attributes[0]));
        return dataPoint;
    }
}