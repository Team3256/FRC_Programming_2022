package frc.robot.helper.shooter;

public class TrainingDataPoint {
    public double velocityTraining;
    public double exitAngleTraining;
    public double calibratedHoodAngleTraining;
    public double calibratedVelocityTraining;

    public TrainingDataPoint(double vTrain, double thetaTraining, double calibTrainingAngle, double calibTrainingVelocity) {
        velocityTraining = vTrain;
        exitAngleTraining = thetaTraining;
        calibratedHoodAngleTraining = calibTrainingAngle;
        calibratedVelocityTraining = calibTrainingVelocity;
    }


}
