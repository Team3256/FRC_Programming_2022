package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonConfiguration;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.helper.shooter.TrainingDataPoint;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.PiecewiseBicubicSplineInterpolatingFunction;
import org.apache.commons.math3.analysis.interpolation.PiecewiseBicubicSplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;
import frc.robot.helper.linearalgebra.Vector;
import frc.robot.helper.shooter.DininnoDataPoint;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase {
    private static final RobotLogger logger = new RobotLogger(ShooterSubsystem.class.getCanonicalName());

    public enum ShooterLocationPreset {
        LAUNCHPAD,
        TARMAC_VERTEX,
    }

    private double targetVelocity = 0;
    private final TalonFX masterLeftShooterMotor;
    private final TalonFX followerRightShooterMotor;

    private final TalonFX hoodAngleMotor;
    private final DigitalInput limitSwitch;
//    private double currentSetpoint = 0;

    private ShooterLocationPreset shooterLocationPreset = ShooterLocationPreset.TARMAC_VERTEX;

    private PiecewiseBicubicSplineInterpolatingFunction velocityInterpolatingFunction;
    private PiecewiseBicubicSplineInterpolatingFunction hoodAngleInterpolatingFunction;
    private static PolynomialSplineFunction distanceToHoodAngleInterpolatingFunction;
    private static PolynomialSplineFunction distanceToFlywheelRPMInterpolatingFunction;
    private static PolynomialSplineFunction distanceToTimeInterpolatingFunction;
    private PIDController robotAnglePIDController = new PIDController(ROBOT_ANGLE_PID_P, ROBOT_ANGLE_PID_I, ROBOT_ANGLE_PID_D);
    private PIDController hoodAnglePIDController = new PIDController(HOOD_ANGLE_PID_P, HOOD_ANGLE_PID_I, HOOD_ANGLE_PID_D);
    private PIDController flywheelRPMPIDController = new PIDController(FLYWHEEL_RPM_PID_P, FLYWHEEL_RPM_PID_I, FLYWHEEL_RPM_PID_D);
    static {
        double[] trainDistance = new double[TRAINING_DATA.size()];
        double[] trainTime = new double[TRAINING_DATA.size()];
        double[] trainHoodAngle = new double[TRAINING_DATA.size()];
        double[] trainFlywheelRPM = new double[TRAINING_DATA.size()];
        for(int i = 0; i < TRAINING_DATA.size(); i++) {
            DininnoDataPoint dataPoint = TRAINING_DATA.get(i);
            trainDistance[i] = dataPoint.distance;
            trainTime[i] = dataPoint.time;
            trainHoodAngle[i] = dataPoint.hoodAngle;
            trainFlywheelRPM[i] = dataPoint.flywheelRPM;
        }

        distanceToFlywheelRPMInterpolatingFunction = new LinearInterpolator().interpolate(trainDistance, trainFlywheelRPM);
        distanceToHoodAngleInterpolatingFunction = new LinearInterpolator().interpolate(trainDistance, trainHoodAngle);
        distanceToTimeInterpolatingFunction = new LinearInterpolator().interpolate(trainDistance, trainTime);
    }


    public ShooterSubsystem() {
        TalonConfiguration MASTER_CONFIG = new TalonConfiguration();
        MASTER_CONFIG.NEUTRAL_MODE = NeutralMode.Coast;
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
                masterLeftShooterMotor,
                FOLLOWER_CONFIG,
                MANI_CAN_BUS
        );


        TalonConfiguration hoodConfig = new TalonConfiguration(new TalonConfiguration.TalonFXPIDFConfig(1,0,10,0), InvertType.None, NeutralMode.Brake);

        hoodAngleMotor = TalonFXFactory.createTalonFX(HOOD_MOTOR_ID, hoodConfig, MANI_CAN_BUS);
        limitSwitch = new DigitalInput(HOOD_LIMITSWITCH_CHANNEL);

        logger.info("Flywheel Initialized");
    }

    public void setTargetVelocity(double targetVelocity) {
        this.targetVelocity = targetVelocity;
    }

    public double getTargetVelocity() {
        return this.targetVelocity;
    }

    /**
     * @param percent Velocity from min to max as percent from xbox controller (0% - 100%)
     * Flywheel speed is set by integrated get controller
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
        hoodAngleMotor.neutralOutput();
    }
    /**
     * reverses the hood for zeroing the hood motor
     */
    public void hoodSlowReverse(){
        System.out.println("Slow Reverse Hood");
        hoodAngleMotor.set(ControlMode.PercentOutput, HOOD_SLOW_REVERSE_PERCENT);
    }
    /**
     * zeros the hood motor sensor
     */
    public void zeroHoodMotorSensor(){
        hoodAngleMotor.setSelectedSensorPosition(0);
    }
    /**
     * checks if limit switch is pressed
     */
    public boolean isHoodLimitSwitchPressed(){
        return !limitSwitch.get();
    }
    /**
     * Disables powers to flywheel motor, motors change to neutral/coast mode
     */
    public void stopFlywheel() {
        masterLeftShooterMotor.neutralOutput();
    }

    public void setShooterLocationPreset(ShooterLocationPreset preset) {
        SmartDashboard.putString("Shooter Preset: ", preset.toString());
        logger.info("Shooter Preset Changed to " + preset);
        this.shooterLocationPreset = preset;
    }

    /*
     * Confirms if velocity is within margin of set point
     */

    public boolean isAtSetPoint(double setpoint) {
        double velocity = getFlywheelRPM();

        return (velocity <= setpoint + SET_POINT_ERROR_MARGIN*setpoint) &&
                (velocity >= setpoint - SET_POINT_ERROR_MARGIN*setpoint);
    }
    public boolean isAtSetPoint(DoubleSupplier setpoint) {
        double velocity = getFlywheelRPM();

        return (velocity <= setpoint.getAsDouble() + SET_POINT_ERROR_MARGIN*setpoint.getAsDouble()) &&
                (velocity >= setpoint.getAsDouble() - SET_POINT_ERROR_MARGIN*setpoint.getAsDouble());
    }

    private double getAngularVelocityFromCalibration(double ballVelocity, double ballAngle) {
        if(velocityInterpolatingFunction == null){
            logger.warning("Velocity Interpolating Function is NULL");
        }
        return velocityInterpolatingFunction.value(ballVelocity, ballAngle);
    }

    private void getVelocityInterpolatingFunctionFromPoints(){

        double[] vValTrain = new double[ALL_SHOOTER_CALIB_TRAINING.size()];
        double[] thetaValTrain = new double[ALL_SHOOTER_CALIB_TRAINING.size()];
        double[][] angularVelocityTrain = new double[ALL_SHOOTER_CALIB_TRAINING.size()][ALL_SHOOTER_CALIB_TRAINING.size()];

        TrainingDataPoint data;
        for (int i = 0; i < ALL_SHOOTER_CALIB_TRAINING.size(); i++) {
            data = ALL_SHOOTER_CALIB_TRAINING.get(i);
            vValTrain[i] = data.velocityTraining;
            thetaValTrain[i] = data.exitAngleTraining;
            angularVelocityTrain[i][i] = data.flywheelRPM;
        }

        velocityInterpolatingFunction = new PiecewiseBicubicSplineInterpolator()
                .interpolate(vValTrain, thetaValTrain, angularVelocityTrain);
    }

    private void getHoodAngleInterpolatingFunctionFromPoints(){
        double[] vValTrain = new double[ALL_SHOOTER_CALIB_TRAINING.size()];
        double[] thetaValTrain = new double[ALL_SHOOTER_CALIB_TRAINING.size()];
        double[][] hoodValTrain = new double[ALL_SHOOTER_CALIB_TRAINING.size()][ALL_SHOOTER_CALIB_TRAINING.size()];

        TrainingDataPoint data;
        for (int i = 0; i < ALL_SHOOTER_CALIB_TRAINING.size(); i++) {
            data = ALL_SHOOTER_CALIB_TRAINING.get(i);
            vValTrain[i] = data.velocityTraining;
            thetaValTrain[i] = data.exitAngleTraining;
            hoodValTrain[i][i] = data.calibratedHoodAngleTraining;
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

    public double getHoodAngleFromInterpolator(double distance) {
        if(distanceToHoodAngleInterpolatingFunction == null){
            logger.warning("Distance to Hood Angle Interpolation Function is NULL");
        }

        return distanceToHoodAngleInterpolatingFunction.value(boundDistanceToInterpolation(distance));
    }

    public double getFlywheelRPMFromInterpolator(double distance) {
        if(distanceToFlywheelRPMInterpolatingFunction == null){
            logger.warning("Distance to Flywheel RPM Interpolation Function is NULL");
        }

        return distanceToFlywheelRPMInterpolatingFunction.value(boundDistanceToInterpolation(distance));
    }

    private double boundDistanceToInterpolation(double distance) {
        double lowerBounded = Math.max(distance, 60); // lower bound
        double bounded = Math.min(distance, 204); // upper bound
        return bounded;
    }

    /**
     * @return current velocity of motors
     */
    private double getVelocity() {
        double velocityInSensorUnits = masterLeftShooterMotor.getSensorCollection().getIntegratedSensorVelocity();
        return fromSuToRPM(velocityInSensorUnits) ; // su / 100ms  * 1/2048 * 10 100ms/ 1s 60s / min
    }

    public static double fromSuToRPM(double su){
        return su  * (10 * 60) / 2048;
    }


    public double getFlywheelRPM(){
        return this.fromSuToRPM(masterLeftShooterMotor.getSelectedSensorVelocity());

    }

    public ShooterState getFlywheelShooterStateFromPreset(){
        return ALL_SHOOTER_PRESETS.get(shooterLocationPreset).shooterState;
    }

    @Override
    public void periodic() {
        NetworkTableInstance.getDefault().getTable("Debug").getEntry("HOOD Limit").setBoolean( this.isHoodLimitSwitchPressed());
    }

    public double[] dininnoAlgorithm(double distance, double robotXVelocity, double robotYVelocity) {
        double ti = t0;
        Vector d = new Vector(0, distance);
        Vector robotVelocity = new Vector(robotXVelocity, robotYVelocity);
        double magRi = calculate(ti, robotVelocity, d);
        while (magRi > dininnoConstant * RADIUS_UPPER_HUB) {
            double beta = dininnoConstant2 * Math.sqrt(magRi);
            double tiMinus = ti - beta;
            double tiPlus = ti + beta;
            if (calculate(tiMinus, robotVelocity, d) > calculate(tiPlus, robotVelocity, d)) {
                ti = tiPlus;
            }
            else if (calculate(tiMinus, robotVelocity, d) < calculate(tiPlus, robotVelocity, d)) {
                ti = tiMinus;
            }
            else if (calculate(tiMinus, robotVelocity, d) == calculate(tiPlus, robotVelocity, d)) {
                ti = ti + 1/2 * beta;
            }
        }

        double sn = magRi - distance;
        double alpha = Math.acos( (-Math.pow(magRi, 2) + sn + Math.pow(distance, 2)) / (2 * sn * distance) );

        double omega = distanceToFlywheelRPMInterpolatingFunction.value(sn);
        double theta = distanceToHoodAngleInterpolatingFunction.value(sn);

        double calculatedValues[] = new double[3];
        calculatedValues[0] = omega; //Flywheel RPM
        calculatedValues[1] = theta; //Hood Angle
        calculatedValues[2] = alpha; //Robot Angle

        return calculatedValues;
    }

    public double calculate(double time, Vector robotVelocity, Vector distance) {
        Vector ri = Vector.multiply(robotVelocity, time);
        Vector sn = Vector.add(ri, distance);
        double magSn = sn.magnitude();
        double tn = distanceToTimeInterpolatingFunction.value(magSn);
        double magRiPrime = Vector.magnitude(Vector.multiply(robotVelocity, (time - tn)));
        return magRiPrime;
    }

    public double adjustRobotAngle(double currentAngle, double targetAngle) {
        return robotAnglePIDController.calculate(targetAngle, currentAngle);
    }

    public double adjustFlywheelRPM(double currentRPM, double targetRPM) {
        return flywheelRPMPIDController.calculate(targetRPM, currentRPM);
    }

    public double adjustHoodAngle(double currentAngle, double targetAngle) {
        return hoodAnglePIDController.calculate(targetAngle, currentAngle);
    }
}