package frc.robot.helper.shooter;

public class TrainingDataPoint {
    public double distance;
    public double hoodAngle;
    public double velocityTraining;
    public double exitAngleTraining;
    public double calibratedHoodAngleTraining;
    public double flywheelRPM;
    public double time;


    public TrainingDataPoint(double distance, double hoodAngle, double flywheelRPM) {
        this.distance = distance;
        this.hoodAngle = hoodAngle;
        this.flywheelRPM = flywheelRPM;
    }

    public TrainingDataPoint(double distance, double hoodAngle, double flywheelRPM, double time) {
        this.distance = distance;
        this.hoodAngle = hoodAngle;
        this.flywheelRPM = flywheelRPM;
        this.time = time;
    }

}
