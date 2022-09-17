package frc.robot.helper.shooter;

import frc.robot.subsystems.ShooterSubsystem;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.util.function.DoubleSupplier;

import static frc.robot.Constants.ShooterConstants.*;

public class ShootingWhileMovingHelper {
    public static class ShootingWhileMovingState {
        public double distance = 0;
        public double alpha = 0;
        public boolean readyToShoot = false;

        public ShootingWhileMovingState(double distance, double alpha, boolean readyToShoot) {
            this.distance = distance;
            this.alpha = alpha;
            this.readyToShoot = readyToShoot;
        }
    }

    private ShooterSubsystem shooterSubsystem;

    private DoubleSupplier distanceSupplier;
    private DoubleSupplier velocityXSupplier;
    private DoubleSupplier velocityYSupplier;

    private int maxIterations = 50;
    private final double kP = 0.05;

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
        double alpha = alphaGuess; // radians
        double error = Double.POSITIVE_INFINITY;
        double predictedDistanceToHub = 0;

        if (Math.hypot(velocityXSupplier.getAsDouble(), velocityYSupplier.getAsDouble()) > SHOOTING_WHILE_MOVING_THRESHOLD) {
            // if we are moving slow
            // no need to use shooting while moving
            // so just shoot normally
            return new ShootingWhileMovingState(distanceSupplier.getAsDouble(), 0, true); 
        }

        while (iterations < maxIterations && error > TARGET_SHOOTING_WHILE_MOVING_ERROR) {
            double xAim = distanceSupplier.getAsDouble()/((Math.tan(Math.PI/2 - alpha)) - (velocityYSupplier.getAsDouble()/velocityXSupplier.getAsDouble()));
            double yAim = findSolutionLineY(xAim);

            //finding time to get to predicted dn
            predictedDistanceToHub = Math.sqrt(Math.pow(xAim, 2) + Math.pow(yAim, 2));
            double predictedTimeToHub = shooterSubsystem.getTimeFromInterpolator(predictedDistanceToHub);

            error = calculateError(predictedTimeToHub, xAim, yAim);
            alpha = calculateNextAlpha(error, alpha);
        }

        return new ShootingWhileMovingState(predictedDistanceToHub, alpha, error < TARGET_SHOOTING_WHILE_MOVING_ERROR);
    }

    private double findSolutionLineY(double x) {
        return (velocityYSupplier.getAsDouble()/velocityXSupplier.getAsDouble()) * x + distanceSupplier.getAsDouble();
    }

    public double calculateNextAlpha(double error, double alpha){
        return error*-kP + alpha;
    }

    private double calculateError(double time, double x, double y) {
        return Math.copySign(
            Math.sqrt(
                Math.pow(
                    (time * -velocityXSupplier.getAsDouble() - x),
                    2
                ) + 
                Math.pow(
                    ((time * -velocityYSupplier.getAsDouble() + distanceSupplier.getAsDouble()) - y),
                    2
                )
            ),
            time * velocityXSupplier.getAsDouble() - x
        );
    }
}
