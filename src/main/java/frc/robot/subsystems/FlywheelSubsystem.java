package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonConfiguration;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.CSVShooting.ReadTrainingFromCSV;
import frc.robot.helper.CSVShooting.TrainingDataPoint;
import org.apache.commons.math3.analysis.interpolation.*;
import java.util.List;
import java.util.logging.Logger;

import static frc.robot.Constants.HangerConstants.*;
import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class FlywheelSubsystem extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(FlywheelSubsystem.class.getCanonicalName());

    private final TalonFX masterLeftShooterMotor;
    private final TalonFX followerRightShooterMotor;

    private final TalonFX hoodAngleMotor;
    private final DigitalInput limitSwitch;

    private double currentTargetSpeed;

    private List<TrainingDataPoint> velocityTrainingPoints;
    private List<TrainingDataPoint> hoodAngleTrainingPoints;

    private PiecewiseBicubicSplineInterpolatingFunction velocityInterpolatingFunction;
    private PiecewiseBicubicSplineInterpolatingFunction hoodAngleInterpolatingFunction;

    public FlywheelSubsystem() {
        TalonConfiguration MASTER_CONFIG = new TalonConfiguration();
        MASTER_CONFIG.NEUTRAL_MODE = NeutralMode.Brake;
        MASTER_CONFIG.INVERT_TYPE = InvertType.InvertMotorOutput;
        MASTER_CONFIG.PIDF_CONSTANTS = new TalonConfiguration.TalonFXPIDFConfig(
                SHOOTER_MASTER_TALON_PID_P,
                SHOOTER_MASTER_TALON_PID_I,
                SHOOTER_MASTER_TALON_PID_D,
                SHOOTER_MASTER_TALON_PID_F
        );

        TalonConfiguration FOLLOWER_CONFIG = TalonConfiguration.createFollowerConfig(MASTER_CONFIG, InvertType.OpposeMaster);

        masterLeftShooterMotor = TalonFXFactory.createTalonFX(
                PID_SHOOTER_MOTOR_ID_LEFT,
                MASTER_CONFIG,
                MANI_CAN_BUS
        );
        followerRightShooterMotor = TalonFXFactory.createFollowerTalonFX(PID_SHOOTER_MOTOR_ID_RIGHT,
                PID_SHOOTER_MOTOR_ID_RIGHT,
                FOLLOWER_CONFIG,
                MANI_CAN_BUS
        );

        hoodAngleMotor = new TalonFX(HOOD_MOTOR_ID);
        limitSwitch = new DigitalInput(HOOD_LIMITSWITCH_CHANNEL);

        logger.info("Flywheel Initialized");
      
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
     * @param hoodAngle motor units
     * motor moves to hoodAngle position
     */
    public void setHoodAngle(double hoodAngle) {
        hoodAngleMotor.set(ControlMode.Position, hoodAngle);
    }
    /**
     * stops the hood motor
     */
    public void stopHood(){
        hoodAngleMotor.set(ControlMode.PercentOutput, 0);
    }
    /**
     * reverses the hood for zeroing the hood motor
     */
    public void hoodSlowReverse(){
        hoodAngleMotor.set(ControlMode.PercentOutput, HOOD_SLOW_REVERSE_PERCENT);
    }
    /**
     * zeros the hood motor sensor
     */
    public void zeroHoodMotor(){
        hoodAngleMotor.setSelectedSensorPosition(0);
    }
    /**
     * checks if limit switch is pressed
     */
    public boolean isHoodLimitSwitchPressed(){
        return limitSwitch.get();
    }
    /**
     * Disables powers to flywheel motor, motors change to neutral/coast mode
     */
    public void stopFlywheel() {
        masterLeftShooterMotor.neutralOutput();
    }

    /**
     * Disables both the shooter hood and motors
     */
    public void stopFullShooter() {
        stopFlywheel();
        stopHood();
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
        if(velocityInterpolatingFunction == null){
            logger.warning("Velocity Interpolating Function is NULL");
        }
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

        velocityInterpolatingFunction = new PiecewiseBicubicSplineInterpolator()
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

        hoodAngleInterpolatingFunction = new PiecewiseBicubicSplineInterpolator()
                .interpolate(vValTrain, thetaValTrain, hoodValTrain);

    }

    private double getHoodValueFromCalibration(double ballVelocity, double ballAngle) {
        if(hoodAngleInterpolatingFunction == null){
            logger.warning("Hood Angle Interpolation Function is NULL");
        }
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