package frc.robot.helper.shooter;

import frc.robot.subsystems.ShooterSubsystem;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.function.DoubleSupplier;
import static frc.robot.Constants.ShooterConstants.EPSILON;
import static frc.robot.Constants.ShooterConstants.SHOOTER_DATA;

public class ShootingWhileMovingHelper {
    public static class ShootingWhileMovingState {
        public double distance = 0;
        public double alpha = 0;

        public ShootingWhileMovingState(double distance, double alpha) {
            this.distance = distance;
            this.alpha = 0;
        }
    }

    private ShooterSubsystem shooterSubsystem;

    private DoubleSupplier distanceSupplier;
    private DoubleSupplier velocityXSupplier;
    private DoubleSupplier velocityYSupplier;

    private static PolynomialSplineFunction distanceToTime;

    private int maxIterations = 100;
    private final double kP = 0.05;

    static {
        double[] trainDistance = new double[SHOOTER_DATA.size()];
        double[] trainTime = new double[SHOOTER_DATA.size()];
        for (int i = 0; i < SHOOTER_DATA.size(); i++) {
            TrainingDataPoint dataPoint = SHOOTER_DATA.get(i);
            trainDistance[i] = dataPoint.distance;
            trainTime[i] = dataPoint.time;
        }

        distanceToTime = new LinearInterpolator().interpolate(trainDistance, trainTime);
    }

    public ShootingWhileMovingHelper(ShooterSubsystem shooterSubsystem, DoubleSupplier distance, DoubleSupplier vX, DoubleSupplier vY) {
        this.shooterSubsystem = shooterSubsystem;
        this.distanceSupplier = distance;
        this.velocityXSupplier = vX;
        this.velocityYSupplier = vY;
    }

    public ShootingWhileMovingHelper(ShooterSubsystem shooterSubsystem, DoubleSupplier distance, DoubleSupplier vX, DoubleSupplier vY, int maxIterations) {
        this.shooterSubsystem = shooterSubsystem;
        this.distanceSupplier = distance;
        this.velocityXSupplier = vX;
        this.velocityYSupplier = vY;
        this.maxIterations = maxIterations;
    }

    public ShootingWhileMovingState calculate() {
        return calculate(0.1);
    }

    public ShootingWhileMovingState calculate(double alphaGuess) {
        int iterations = 0;
        double alpha = 0.1; // radians
        double error = Double.POSITIVE_INFINITY;
        double predictedDistanceToHub = 0;

        while (iterations < maxIterations && error > EPSILON) {
            double xAim = distanceSupplier.getAsDouble()/((Math.tan(Math.PI/2 - alpha)) - (velocityYSupplier.getAsDouble()/velocityXSupplier.getAsDouble()));
            double yAim = findSolutionLineY(xAim);

            //finding time to get to predicted dn
            predictedDistanceToHub = Math.sqrt(Math.pow(xAim, 2) + Math.pow(yAim, 2));
            double predictedTimeToHub = shooterSubsystem.getFlywheelRPMFromInterpolator(predictedDistanceToHub);

            error = calculateError(predictedTimeToHub, xAim, yAim);
            alpha = calculateNextAlpha(error, alpha);
        }

        return new ShootingWhileMovingState(predictedDistanceToHub, alpha);
    }

    private double findSolutionLineY(double x) {
        return (velocityYSupplier.getAsDouble()/velocityXSupplier.getAsDouble()) * x + distanceSupplier.getAsDouble();
    }

    public double calculateNextAlpha(double error, double alpha){
        return error*-kP + alpha;
    }

    private double calculateError(double time, double x, double y) {
        return Math.copySign(
                    Math.sqrt(Math.pow((time * -velocityXSupplier.getAsDouble() - x), 2) + Math.pow(((time * -velocityYSupplier.getAsDouble() + distanceSupplier.getAsDouble()) - y), 2)),
                    time * velocityXSupplier.getAsDouble() - x
                );
    }
}