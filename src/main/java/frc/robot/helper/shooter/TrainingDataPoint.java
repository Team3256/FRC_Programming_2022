package frc.robot.helper.shooter;

public class TrainingDataPoint {
    public double velocityTraining;
    public double exitAngleTraining;
    public double calibratedHoodAngleTraining;
    public double calibratedVelocityTraining;

    public TrainingDataPoint(double exitBallVelocity, double exitBallAngle, double hoodAngle, double flywheelRPM) {
        velocityTraining = exitBallVelocity;
        exitAngleTraining = exitBallAngle;
        calibratedHoodAngleTraining = hoodAngle;
        calibratedVelocityTraining = flywheelRPM;
    }


}
