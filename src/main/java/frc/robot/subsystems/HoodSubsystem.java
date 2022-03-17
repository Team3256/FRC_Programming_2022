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
import frc.robot.helper.logging.RobotLogger;
import frc.robot.helper.shooter.ShooterPreset;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.helper.shooter.TrainingDataPoint;
import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.PiecewiseBicubicSplineInterpolatingFunction;
import org.apache.commons.math3.analysis.interpolation.PiecewiseBicubicSplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;
import static frc.robot.Constants.ShooterConstants.ALL_SHOOTER_CALIB_TRAINING;

public class HoodSubsystem extends SubsystemBase {
    private final RobotLogger logger = new RobotLogger(this.getName());

    private final TalonFX hoodAngleMotor;
    private PiecewiseBicubicSplineInterpolatingFunction hoodAngleInterpolatingFunction;
    private PolynomialSplineFunction distanceToHoodAngleInterpolatingFunction;
    private final DigitalInput limitSwitch;
    private double zeroPoint = 0;


    public HoodSubsystem(){
        TalonConfiguration hoodConfig = new TalonConfiguration(new TalonConfiguration.TalonFXPIDFConfig(1,0,10,0), InvertType.None, NeutralMode.Brake);

        hoodAngleMotor = TalonFXFactory.createTalonFX(HOOD_MOTOR_ID, hoodConfig, MANI_CAN_BUS);
        limitSwitch = new DigitalInput(HOOD_LIMITSWITCH_CHANNEL);
    }
    /**
     * @param hoodAngle motor units
     * motor moves to hoodAngle position
     */
    public void setHoodAngle(double hoodAngle) {
        hoodAngleMotor.set(ControlMode.Position, hoodAngle - zeroPoint);
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
    public void zeroHoodMotor(){
        zeroPoint = hoodAngleMotor.getSelectedSensorPosition();
    }
    /**
     * checks if limit switch is pressed
     */
    public boolean isHoodLimitSwitchPressed(){
        return !limitSwitch.get();
    }

    /**
     * Disables both the shooter hood and motors
     */

    public double getHoodAngleFromInterpolator(double distance) {
        if(distanceToHoodAngleInterpolatingFunction == null){
            logger.warning("Distance to Hood Angle Interpolation Function is NULL");
        }
        return distanceToHoodAngleInterpolatingFunction.value(distance);
    }

    @Override
    public void periodic(){
        SmartDashboard.putBoolean("Hood Zero Limit Switch", this.isHoodLimitSwitchPressed());
    }

    public double getHoodValueFromCalibration(double ballVelocity, double ballAngle) {
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

    private void trainDistanceToHoodAngleInterpolator() {
        double[] trainDistance = new double[SIMPLE_CALIB_TRAINING.size()];
        double[] trainHoodAngle = new double[SIMPLE_CALIB_TRAINING.size()];
        for(int i = 0; i < SIMPLE_CALIB_TRAINING.size(); i++) {
            TrainingDataPoint dataPoint = SIMPLE_CALIB_TRAINING.get(i);
            trainDistance[i] = dataPoint.distance;
            trainHoodAngle[i] = dataPoint.hoodAngle;
        }
        distanceToHoodAngleInterpolatingFunction = new LinearInterpolator().interpolate(trainDistance, trainHoodAngle);
    }
}
