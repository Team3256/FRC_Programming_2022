package frc.robot.helper.CSVShooting;

public class TrainingDataPoint {
    public double velocityTraining;
    public double exitAngleTraining;
    public double calibratedTraining;

    public TrainingDataPoint(double vTrain, double thetaTraining, double calibTraining) {
        velocityTraining = vTrain;
        exitAngleTraining = thetaTraining;
        calibratedTraining = calibTraining;
    }


}
