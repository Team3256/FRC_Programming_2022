package frc.robot.helper.shooter;

public class DininnoDataPoint {
    public double distance;
    public double time;
    public double hoodAngle;
    public double flywheelRPM;

    public DininnoDataPoint() {
        distance = 0;
        time = 0;
        hoodAngle = 0;
        flywheelRPM = 0;
    }

    public DininnoDataPoint(double distance, double time, double hoodAngle, double flywheelRPM) {
        this.distance = distance;
        this.time = time;
        this.hoodAngle = hoodAngle;
        this.flywheelRPM = flywheelRPM;
    }

}