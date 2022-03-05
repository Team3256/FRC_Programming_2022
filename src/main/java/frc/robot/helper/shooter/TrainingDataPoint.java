package frc.robot.helper.shooter;

public class TrainingDataPoint {
    public double distance;
    public double hoodAngle;
    public double velocityTraining;
    public double exitAngleTraining;
    public double calibratedHoodAngleTraining;
    public double flywheelRPM;

    public TrainingDataPoint(double exitBallVelocity, double exitBallAngle, double hoodAngle, double flywheelRPM) {
        velocityTraining = exitBallVelocity;
        exitAngleTraining = exitBallAngle;
        calibratedHoodAngleTraining = hoodAngle;
        this.flywheelRPM = flywheelRPM;
    }

    public TrainingDataPoint(double distance, double hoodAngle, double flywheelRPM) {
        this.distance = distance;
        this.hoodAngle = hoodAngle;
        this.flywheelRPM = flywheelRPM;
    }


}
