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
import frc.robot.helper.shooter.ShooterPresetSelector;
import frc.robot.helper.shooter.TrainingDataPoint;
import frc.robot.helper.logging.RobotLogger;
import frc.robot.helper.shooter.ShooterPreset;
import frc.robot.helper.shooter.ShooterState;
import org.apache.commons.math3.analysis.interpolation.*;
import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import static frc.robot.Constants.IDConstants.*;
import static frc.robot.Constants.ShooterConstants.*;

public class FlywheelSubsystem extends SubsystemBase {
    public enum ShooterLocationPreset {
        FENDER,
        TARMAC_SIDE_VERTEX,
        TARMAC_MIDDLE_VERTEX,
        TRUSS
    }

    private int currentPresetNumber = 0;

    private static final RobotLogger logger = new RobotLogger(FlywheelSubsystem.class.getCanonicalName());

    private final TalonFX masterLeftShooterMotor;
    private final TalonFX followerRightShooterMotor;

    private final TalonFX hoodAngleMotor;
    private final DigitalInput limitSwitch;

    private double zeroPoint = 0;

    private double currentTargetSpeed;

    private ShooterLocationPreset shooterLocationPreset = ShooterLocationPreset.FENDER;

    private PiecewiseBicubicSplineInterpolatingFunction velocityInterpolatingFunction;
    private PiecewiseBicubicSplineInterpolatingFunction hoodAngleInterpolatingFunction;
    private PolynomialSplineFunction distanceToHoodAngleInterpolatingFunction;
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


        TalonConfiguration hoodConfig = new TalonConfiguration(new TalonConfiguration.TalonFXPIDFConfig(1,0,10,0), InvertType.None, NeutralMode.Brake);

        hoodAngleMotor = TalonFXFactory.createTalonFX(HOOD_MOTOR_ID, hoodConfig, MANI_CAN_BUS);
        limitSwitch = new DigitalInput(HOOD_LIMITSWITCH_CHANNEL);

        logger.info("Flywheel Initialized");

      
       // getVelocityInterpolatingFunctionFromPoints();
        //getHoodAngleInterpolatingFunctionFromPoints();

    }

    private ShooterPreset getPreset() {
        return ALL_SHOOTER_PRESETS.get(currentPresetNumber);
    }

    /**
     * @param distance distance to hoop
     */
    public void autoAim(double distance) {
        ShooterState ikShooterState = ballInverseKinematics(distance);

        ShooterState correctedShooterState = new ShooterState(
                getAngularVelocityFromCalibration(ikShooterState.rpmVelocity, ikShooterState.hoodAngle),
                getHoodValueFromCalibration(ikShooterState.rpmVelocity, ikShooterState.hoodAngle));

        applyShooterState(correctedShooterState);
    }

    public void simpleAutoAim(double distance) {
        ShooterState shooterState = new ShooterState(getFlywheelRPMFromInterpolator(distance), getHoodAngleFromInterpolator(distance));
        applyShooterState(shooterState);
    }

    public void autoPresetAutoAim(double distance) {
        ShooterPreset preset = ShooterPresetSelector.findClosesPreset(distance);
        applyShooterState(preset.shooterState);
    }

    public void setShooterLocationPreset(ShooterLocationPreset preset) {
        SmartDashboard.putString("Shooter Preset: ",preset.toString());
        logger.info("Shooter Preset Changed to " + preset);
        this.shooterLocationPreset = preset;
    }

    public ShooterLocationPreset getShooterLocationPreset() {
        return this.shooterLocationPreset;
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
        setSpeed(fromRpmToSu(shooterState.rpmVelocity));
        setHoodAngle(shooterState.hoodAngle);
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

    private void trainDistanceToFlywheelRPMInterpolator() {
        double[] trainDistance = new double[SIMPLE_CALIB_TRAINING.size()];
        double[] trainFlywheelRPM = new double[SIMPLE_CALIB_TRAINING.size()];
        for(int i = 0; i < SIMPLE_CALIB_TRAINING.size(); i++) {
            TrainingDataPoint dataPoint = SIMPLE_CALIB_TRAINING.get(i);
            trainDistance[i] = dataPoint.distance;
            trainFlywheelRPM[i] = dataPoint.flywheelRPM;
        }
        distanceToHoodAngleInterpolatingFunction = new LinearInterpolator().interpolate(trainDistance, trainFlywheelRPM);
    }

    public double getHoodAngleFromInterpolator(double distance) {
        if(distanceToHoodAngleInterpolatingFunction == null){
            logger.warning("Distance to Hood Angle Interpolation Function is NULL");
        }
       return distanceToHoodAngleInterpolatingFunction.value(distance);
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

    private double fromSuToRPM(double su){
        return su  * (10 * 60) / 2048;
    }

    private double fromRpmToSu(double rpm){
        return rpm  * 2048 / (10 * 60) ;
    }

    public void increasePreset() {
        currentPresetNumber += 1;
        if (currentPresetNumber >= ALL_SHOOTER_PRESETS.size()) {
            currentPresetNumber = 0;
        }

        SmartDashboard.putString("Preset: ", getPreset().presetName);
    }

    public void decreasePreset() {
        currentPresetNumber -= 1;
        if (currentPresetNumber == -1) {
            currentPresetNumber = ALL_SHOOTER_PRESETS.size();
        }

        SmartDashboard.putString("Preset: ", getPreset().presetName);
    }

    public void shootSelectedPreset() {
        ShooterPreset currentPreset = getPreset();
        this.setSpeed(fromRpmToSu(currentPreset.shooterState.rpmVelocity));
        this.setHoodAngle(currentPreset.shooterState.hoodAngle);
    }

    public double getFlywheelRPM(){
        return this.fromSuToRPM(masterLeftShooterMotor.getSelectedSensorVelocity());

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Flywheel RPM", masterLeftShooterMotor.getSelectedSensorVelocity());
    }
}

