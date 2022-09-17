package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.hardware.TalonConfiguration;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.helper.shooter.ShooterState;
import frc.robot.helper.shooter.TrainingDataPoint;
import io.github.oblarg.oblog.Loggable;

import org.apache.commons.math3.analysis.interpolation.LinearInterpolator;
import org.apache.commons.math3.analysis.interpolation.PiecewiseBicubicSplineInterpolatingFunction;
import org.apache.commons.math3.analysis.interpolation.PiecewiseBicubicSplineInterpolator;
import org.apache.commons.math3.analysis.interpolation.SplineInterpolator;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import java.math.BigDecimal;
import java.util.function.DoubleSupplier;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class ShooterSubsystem extends SubsystemBase implements Loggable {
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

    BigDecimal KF_PERCENT_FACTOR_FLYWHEEL = new BigDecimal("0.00018082895");
    BigDecimal KF_CONSTANT = new BigDecimal("0.0159208876");

    private static final PolynomialSplineFunction distanceToHoodAngleInterpolatingFunction;
    private static final PolynomialSplineFunction distanceToFlywheelRPMInterpolatingFunction;
    private static final PolynomialSplineFunction distanceToTimeInterpolatingFunction;

    static {
        double[] trainDistance = new double[SHOOTER_DATA.size()];
        double[] trainFlywheelHood = new double[SHOOTER_DATA.size()];
        double[] trainFlywheelRPM = new double[SHOOTER_DATA.size()];
        double[] trainFlywheelTime = new double[SHOOTER_DATA.size()];
        for(int i = 0; i < SHOOTER_DATA.size(); i++) {
            TrainingDataPoint dataPoint = SHOOTER_DATA.get(i);
            trainDistance[i] = dataPoint.distance;
            trainFlywheelHood[i] = dataPoint.hoodAngle;
            trainFlywheelRPM[i] = dataPoint.flywheelRPM;
            trainFlywheelTime[i] = dataPoint.time;
        }

        distanceToFlywheelRPMInterpolatingFunction = new LinearInterpolator().interpolate(trainDistance, trainFlywheelRPM);
        distanceToHoodAngleInterpolatingFunction = new LinearInterpolator().interpolate(trainDistance, trainFlywheelHood);
        distanceToTimeInterpolatingFunction = new LinearInterpolator().interpolate(trainDistance, trainFlywheelTime);
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
     * @param RPM of the flywheel (including gearing)
     * Flywheel speed is set by integrated get controller
     */
    public void setVelocity(double rpm) {
        BigDecimal feedforward = (new BigDecimal(targetVelocity).multiply(KF_PERCENT_FACTOR_FLYWHEEL)).add(KF_CONSTANT);

        double feedForwardedPidOutput = rpm + feedforward.doubleValue();

        // Ensure it is never negative
        double positiveMotorOutput = (feedForwardedPidOutput <= 0) ? 0 : feedForwardedPidOutput;
        double clampedPositiveFinalMotorOutput = (positiveMotorOutput > 1) ? 1 : positiveMotorOutput;

        this.setPercentSpeed(clampedPositiveFinalMotorOutput);
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

    /*
     * Confirms if velocity is within margin of set point
     */
    public boolean isAtSetPoint() {
        double velocity = getFlywheelRPM();

        return (velocity <= this.targetVelocity + SET_POINT_ERROR_MARGIN*this.targetVelocity) &&
                (velocity >= this.targetVelocity - SET_POINT_ERROR_MARGIN*this.targetVelocity);
    }

    public double getTimeFromInterpolator(double distance) {
        if(distanceToTimeInterpolatingFunction == null){
            logger.warning("Distance to Time Interpolation Function is NULL");
        }

        return distanceToTimeInterpolatingFunction.value(clampDistanceToInterpolation(distance));
    }

    public double getHoodAngleFromInterpolator(double distance) {
        if(distanceToHoodAngleInterpolatingFunction == null){
            logger.warning("Distance to Hood Angle Interpolation Function is NULL");
        }

        return distanceToHoodAngleInterpolatingFunction.value(clampDistanceToInterpolation(distance));
    }

    public double getFlywheelRPMFromInterpolator(double distance) {
        if(distanceToFlywheelRPMInterpolatingFunction == null){
            logger.warning("Distance to Flywheel RPM Interpolation Function is NULL");
        }

        return distanceToFlywheelRPMInterpolatingFunction.value(clampDistanceToInterpolation(distance));
    }

    private double clampDistanceToInterpolation(double distance) {
        // if limelight is not tracking, we are prob out of range and should set to max distance
        if (distance == 0) return SHOOTER_INTERPOLATION_MAX_VALUE;
        return MathUtil.clamp(distance, SHOOTER_INTERPOLATION_MIN_VALUE, SHOOTER_INTERPOLATION_MAX_VALUE);
    }

    public static double fromSuToRPM(double su){
        return su  * (10 * 60) / 2048;
    }

    public double getFlywheelRPM(){
        return ShooterSubsystem.fromSuToRPM(masterLeftShooterMotor.getSelectedSensorVelocity());
    }

    @Override
    public void periodic() {
        if (Constants.DEBUG) {
            SmartDashboard.putNumber("Actual Flywheel Velocity", getFlywheelRPM());
            SmartDashboard.putNumber("Actual Hood Angle", hoodAngleMotor.getSelectedSensorPosition());
            NetworkTableInstance.getDefault().getTable("Debug").getEntry("HOOD Limit").setBoolean(this.isHoodLimitSwitchPressed());
        }
    }
}

