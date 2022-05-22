package frc.robot.helper.shooter;

import edu.wpi.first.math.controller.PIDController;
import frc.robot.helper.linearalgebra.Vector;
import frc.robot.subsystems.ShooterSubsystem;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ShooterConstants.dininnoConstant2;

public class DininnoShootingAlgorithm {

    private static Vector robotVelocity;
    private static Vector ri;
    private static double magSn;

    public static void dininnoAlgorithm(double distance, double robotXVelocity, double robotYVelocity) {
        double ti = t0;
        robotVelocity = new Vector(robotXVelocity, robotYVelocity);
        Vector d = new Vector(0, distance);
        double magRp = calculate(ti, robotVelocity, d);
        while (magRp > dininnoConstant * RADIUS_UPPER_HUB) {
            double beta = dininnoConstant2 * Math.sqrt(magRp);
            double tiMinus = ti - beta;
            double tiPlus = ti + beta;
            if (calculate(tiMinus, robotVelocity, d) > calculate(tiPlus, robotVelocity, d)) {
                ti = tiPlus;
            }
            else if (calculate(tiMinus, robotVelocity, d) < calculate(tiPlus, robotVelocity, d)) {
                ti = tiMinus;
            }
            else {
                ti = ti + 1/2 * beta;
            }
            magRp = calculate(ti, robotVelocity, d);
        }
    }

    private static double calculate(double time, Vector robotVelocity, Vector distance) {
        ri = Vector.multiply(robotVelocity, time);
        Vector sn = Vector.add(ri, distance);
        magSn = sn.magnitude();
        double tn = ShooterSubsystem.distanceToTimeInterpolatingFunction.value(magSn);
        double magRiPrime = Vector.magnitude(Vector.multiply(robotVelocity, (time - tn)));
        return magRiPrime;
    }

    public static double getPredictedDistance() {
        return magSn;
    }

    public static double getRobotAngle(double distance) {
        double alpha = Math.acos( (-Math.pow(ri.magnitude(), 2) + magSn + Math.pow(distance, 2)) / (2 * magSn * distance) );
        return alpha;
    }
}
