package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonConfiguration;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.shooter.*;
import frc.robot.helper.logging.RobotLogger;
import org.apache.commons.math3.analysis.interpolation.*;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class FlywheelSubsystem extends SubsystemBase {
    
    private static final RobotLogger logger = new RobotLogger(FlywheelSubsystem.class.getCanonicalName());

    private final TalonFX masterLeftShooterMotor;
    private final TalonFX followerRightShooterMotor;

    private double currentTargetSpeed;

    private PiecewiseBicubicSplineInterpolatingFunction velocityInterpolatingFunction;

    private PolynomialSplineFunction distanceToFlywheelRPMInterpolatingFunction;

    public FlywheelSubsystem() {
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

        logger.info("Flywheel Initialized");

      
        //getVelocityInterpolatingFunctionFromPoints();
        //getHoodAngleInterpolatingFunctionFromPoints();

    }

    /**
     * @param velocity Angular Velocity in (rev/s)
     * Flywheel speed is set by integrated PID controller
     */
    public void setSpeed(double velocity) {
        // formula for converting m/s to sensor units/100ms
        currentTargetSpeed = fromRpmToSu(velocity); // rev/s * 1s/10 (100ms) * 2048su/1rev
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
    * Confirms if velocity is within margin of set point
    */
    public boolean isAtSetPoint() {
        double velocity = getVelocity();

        return (velocity <= currentTargetSpeed + SET_POINT_ERROR_MARGIN) &&
                (velocity >= currentTargetSpeed - SET_POINT_ERROR_MARGIN);
    }
    public void stop(){
        setSpeed(0);
    }

    public double getAngularVelocityFromCalibration(double ballVelocity, double ballAngle) {
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
    private void trainDistanceToFlywheelRPMInterpolator() {
        double[] trainDistance = new double[SIMPLE_CALIB_TRAINING.size()];
        double[] trainFlywheelRPM = new double[SIMPLE_CALIB_TRAINING.size()];
        for(int i = 0; i < SIMPLE_CALIB_TRAINING.size(); i++) {
            TrainingDataPoint dataPoint = SIMPLE_CALIB_TRAINING.get(i);
            trainDistance[i] = dataPoint.distance;
            trainFlywheelRPM[i] = dataPoint.flywheelRPM;
        }
        distanceToFlywheelRPMInterpolatingFunction = new LinearInterpolator().interpolate(trainDistance, trainFlywheelRPM);
    }
    public double getFlywheelRPMFromInterpolator(double distance) {
        if(distanceToFlywheelRPMInterpolatingFunction == null){
            logger.warning("Distance to Flywheel RPM Interpolation Function is NULL");
        }
        return distanceToFlywheelRPMInterpolatingFunction.value(distance);
    }

    /**
     * @return current velocity of motors
     */
    private double getVelocity() {
        double velocityInSensorUnits = masterLeftShooterMotor.getSensorCollection().getIntegratedSensorVelocity();
        return fromSuToRPM(velocityInSensorUnits) ; // su / 100ms  * 1/2048 * 10 100ms/ 1s 60s / min
    }
    public double getFlywheelRPM(){
        return this.fromSuToRPM(masterLeftShooterMotor.getSelectedSensorVelocity());
    }

    private double fromSuToRPM(double su){
        return su  * (10 * 60) / 2048;
    }
    private double fromRpmToSu(double rpm){
        return rpm  * 2048 / (10 * 60) ;
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel RPM", masterLeftShooterMotor.getSelectedSensorVelocity());
    }
}

