package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import frc.robot.helper.CSVShooting.ReadTrainingFromCSV;
import frc.robot.helper.CSVShooting.TrainingDataPoint;
import org.apache.commons.math3.analysis.interpolation.*;

import java.util.List;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class FlywheelSubsystem extends SubsystemBase {
    private final TalonFX masterLeftShooterMotor;
    private final TalonFX followerRightShooterMotor;

    private final Servo hoodAngleServo;

    private double currentTargetSpeed;

    private List<TrainingDataPoint> velocityTrainingPoints;
    private List<TrainingDataPoint> hoodAngleTrainingPoints;

    private BicubicSplineInterpolatingFunction velocityInterpolatingFunction;
    private BicubicSplineInterpolatingFunction hoodAngleInterpolatingFunction;

    public FlywheelSubsystem() {
        masterLeftShooterMotor = new TalonFX(PID_SHOOTER_MOTOR_ID_LEFT);
        followerRightShooterMotor = new TalonFX(PID_SHOOTER_MOTOR_ID_RIGHT);

        masterLeftShooterMotor.setInverted(InvertType.InvertMotorOutput);
        followerRightShooterMotor.setInverted(InvertType.None);

        followerRightShooterMotor.follow(masterLeftShooterMotor);

        followerRightShooterMotor.setNeutralMode(NeutralMode.Coast);
        masterLeftShooterMotor.setNeutralMode(NeutralMode.Coast);

        hoodAngleServo = new Servo(HOOD_SERVO_CHANNEL_ID);

        getVelocityInterpolatingFunctionFromPoints();
        getHoodAngleInterpolatingFunctionFromPoints();
    }

    /**
     * @param distance distance to hoop
     */
    public void autoAim(double distance) {
        ShooterState ikShooterState = ballInverseKinematics(distance);

        ShooterState correctedShooterState = new ShooterState(
                getAngularVelocityFromCalibration(ikShooterState.velocity, ikShooterState.theta),
                getHoodValueFromCalibration(ikShooterState.velocity, ikShooterState.theta));

        applyShooterState(correctedShooterState);
    }

    /**
     * @param velocity Angular Velocity in (rev/s)
     * Flywheel speed is set by integrated PID controller
     */
    public void setSpeed(double velocity) {
        // formula for converting m/s to sensor units/100ms
        currentTargetSpeed = velocity * 204.8; // rev/s * 1s/10 (100ms) * 2048su/1rev
        masterLeftShooterMotor.set(ControlMode.Velocity, currentTargetSpeed);
    }

    /**
     * @param percent Velocity from min to max as percent from xbox controller (0% - 100%)
     * Flywheel speed is set by integrated PID controller
     */
    public void setPercentSpeed(double percent) {
        masterLeftShooterMotor.set(ControlMode.PercentOutput, percent);
    }

    /**
     * @param hoodAngle Radians
     * Hood angle set from value 0.0 to 1.0
     */
    public void setHoodAngle(double hoodAngle) { hoodAngleServo.setAngle(hoodAngle); }

    /**
     * Disables powers to motors, motors change to neutral/coast mode
     */
    public void stop() {
        masterLeftShooterMotor.neutralOutput();
    }

    /*
    * Confirms if velocity is within margin of set point
    */
    public boolean isAtSetPoint() {
        double velocity = getVelocity();

        return (velocity <= currentTargetSpeed + SET_POINT_ERROR_MARGIN) &&
                (velocity >= currentTargetSpeed - SET_POINT_ERROR_MARGIN);
    }

    /**
     * @param distance distance from target
     * @return ShooterState with velocity and hood angle settings
     */
    private ShooterState ballInverseKinematics(double distance) {
        double angleEntry = ENTRY_ANGLE_INTO_HUB * Math.PI / 180;

        double distToAimPoint = RADIUS_UPPER_HUB + distance;
        distToAimPoint = distToAimPoint +
                DELTA_DISTANCE_TO_TARGET_FACTOR * distToAimPoint + OFFSET_DISTANCE_FACTOR;

        double deltaHeight = UPPER_HUB_AIMING_HEIGHT - SHOOTER_HEIGHT;
        deltaHeight = deltaHeight +
                DELTA_AIM_HEIGHT_FACTOR * distToAimPoint + OFFSET_HEIGHT_FACTOR;

        double tangentEntryAngle = Math.tan(angleEntry);
        double fourDistHeightTangent = 4 * distToAimPoint * deltaHeight * tangentEntryAngle;
        double distanceToAimSquare = Math.pow(distToAimPoint, 2);
        double deltaHeightSquare = Math.pow(deltaHeight, 2);
        double tangentAimDistSquare = Math.pow(distToAimPoint * tangentEntryAngle, 2);
        double tangentAimDist = distToAimPoint * tangentEntryAngle;


        double exitAngleTheta = -2 * Math.atan((distToAimPoint -
                Math.sqrt(tangentAimDistSquare + fourDistHeightTangent + distanceToAimSquare + 4*deltaHeightSquare))
                / (tangentAimDist + 2 * deltaHeight));
        double velocity = 0.3 * Math.sqrt(54.5) *
                ((Math.sqrt(tangentAimDistSquare + fourDistHeightTangent + distanceToAimSquare + 4*deltaHeightSquare))
                        / Math.sqrt(tangentAimDist + deltaHeight));

        return new ShooterState(velocity, exitAngleTheta);
    }

    /**
     * @param shooterState has both velocity and hood angle
     * sets hood angle and velocity
     */
    private void applyShooterState(ShooterState shooterState) {
        setSpeed(shooterState.velocity);
        setHoodAngle(shooterState.theta);
    }

    private double getAngularVelocityFromCalibration(double ballVelocity, double ballAngle) {
        return velocityInterpolatingFunction.value(ballVelocity, ballAngle);
    }

    private void getVelocityInterpolatingFunctionFromPoints(){
        velocityTrainingPoints = ReadTrainingFromCSV.readDataFromCSV(VEL_CALIB_FILENAME);

        double[] vValTrain = new double[velocityTrainingPoints.size()];
        double[] thetaValTrain = new double[velocityTrainingPoints.size()];
        double[][] angularVelocityTrain = new double[velocityTrainingPoints.size()][velocityTrainingPoints.size()];

        TrainingDataPoint data;
        for (int i = 0; i < velocityTrainingPoints.size(); i++) {
            data = velocityTrainingPoints.get(i);
            vValTrain[i] = data.velocityTraining;
            thetaValTrain[i] = data.exitAngleTraining;
            angularVelocityTrain[i][i] = data.calibratedTraining;
        }

        velocityInterpolatingFunction = new BicubicSplineInterpolator()
                .interpolate(vValTrain, thetaValTrain, angularVelocityTrain);
    }
    private void getHoodAngleInterpolatingFunctionFromPoints(){
        hoodAngleTrainingPoints = ReadTrainingFromCSV.readDataFromCSV(HOOD_CALIB_FILENAME);

        double[] vValTrain = new double[hoodAngleTrainingPoints.size()];
        double[] thetaValTrain = new double[hoodAngleTrainingPoints.size()];
        double[][] hoodValTrain = new double[hoodAngleTrainingPoints.size()][hoodAngleTrainingPoints.size()];

        TrainingDataPoint data;
        for (int i = 0; i < hoodAngleTrainingPoints.size(); i++) {
            data = hoodAngleTrainingPoints.get(i);
            vValTrain[i] = data.velocityTraining;
            thetaValTrain[i] = data.exitAngleTraining;
            hoodValTrain[i][i] = data.calibratedTraining;
        }

        hoodAngleInterpolatingFunction = new BicubicSplineInterpolator()
                .interpolate(vValTrain, thetaValTrain, hoodValTrain);

    }

    private double getHoodValueFromCalibration(double ballVelocity, double ballAngle) {
        double hoodAngle = hoodAngleInterpolatingFunction.value(ballVelocity, ballAngle);
        if (hoodAngle < 0.0) {
            hoodAngle = 0.0;
        } else if (hoodAngle > 1.0) {
            hoodAngle = 1.0;
        }
        return hoodAngle;
    }

    /**
     * @return current velocity of motors
     */
    private double getVelocity() {
        double velocityInSensorUnits = masterLeftShooterMotor.getSensorCollection().getIntegratedSensorVelocity();
        return velocityInSensorUnits  * 10 / 2048;
    }
}

class ShooterState {
    public double velocity;
    public double theta;

    public ShooterState(double v, double t) {
        this.velocity = v;
        this.theta = t;
    }
}