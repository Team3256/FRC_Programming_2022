package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.logging.RobotLogger;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.HangerConstants.*;
import static frc.robot.Constants.IDConstants.*;

public class HangerSubsystem extends SubsystemBase {
    private static final RobotLogger logger = new RobotLogger(HangerSubsystem.class.getCanonicalName());

    private final TalonFX leftTalonMotor;
    private final TalonFX rightTalonMotor;
    private final DoubleSolenoid leftHangerSolenoid;
    private final DoubleSolenoid rightHangerSolenoid;

    DigitalInput bottomLimitSwitch = new DigitalInput(HANGER_LIMITSWITCH_CHANNEL);

    public HangerSubsystem() {
        leftTalonMotor = TalonFXFactory.createTalonFX(
                HANGER_LEFT_MASTER_TALON_ID,
                MASTER_CONFIG,
                MANI_CAN_BUS
        );

        rightTalonMotor = TalonFXFactory.createTalonFX(
                HANGER_RIGHT_FOLLOWER_TALON_ID,
                MASTER_CONFIG,
                MANI_CAN_BUS
        );

        rightTalonMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,90,90,0));
        leftTalonMotor.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true,90,90,0));

        leftHangerSolenoid = new DoubleSolenoid(PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, LEFT_HANGER_SOLENOID_FORWARD, LEFT_HANGER_SOLENOID_BACKWARD);
        rightHangerSolenoid = new DoubleSolenoid(PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, RIGHT_HANGER_SOLENOID_FORWARD, RIGHT_HANGER_SOLENOID_BACKWARD);

        logger.info("Hanger Initialized");

        pneumaticUpright();
    }

    public void extendToHangPosition() {
        logger.info("Extending");
        leftTalonMotor.set(ControlMode.Position, EXTEND_DISTANCE);
        rightTalonMotor.set(ControlMode.Position, EXTEND_DISTANCE);
    }

    public void retractLeftContinuouslyToZero(){
        logger.info("Retracting Left Continuously for zero");

        leftTalonMotor.set(ControlMode.PercentOutput, -1 * HANGER_ZEROING_PERCENT_SPEED);
    }

    public void retractRightContinuouslyToZero(){
        logger.info("Retracting Right Continuously for zero");
        rightTalonMotor.set(ControlMode.PercentOutput, -1 * HANGER_ZEROING_PERCENT_SPEED);
    }

    public void retractToHang(){
        logger.info("Retracting to Hang");
        leftTalonMotor.set(TalonFXControlMode.PercentOutput, -1 * HANGER_RETRACT_PERCENT_SPEED);
        rightTalonMotor.set(TalonFXControlMode.PercentOutput, -1 * HANGER_RETRACT_PERCENT_SPEED);
    }

    public void extendPartial() {
        logger.info("Extending Partially");
        leftTalonMotor.set(ControlMode.Position, PARTIAL_DISTANCE);
        rightTalonMotor.set(ControlMode.Position, PARTIAL_DISTANCE);
    }

    public void pneumaticUpright() {
        leftHangerSolenoid.set(kForward);
        rightHangerSolenoid.set(kForward);
    }

    public void pneumaticSlant() {
        leftHangerSolenoid.set(kReverse);
        rightHangerSolenoid.set(kReverse);
    }

    public void adjustRetract() {

        double distance = ADJUSTMENT_RETRACT_DISTANCE;
        leftTalonMotor.set(ControlMode.Position, distance);
    }
    /**
     * check if the bottom limit switch is triggered
     * @return returns if the bottom limit switch is triggered
     */
    public boolean hasReachedBottom() {
        return bottomLimitSwitch.get();
    }

    public double getLeftPosition() {
        return (leftTalonMotor.getSelectedSensorPosition());
    }


    public double getRightPosition() {
        return (rightTalonMotor.getSelectedSensorPosition());
    }

    /**
     * check if master talon motor has reached the full intended distance-
     * @return returns true if master talon has the full reached intended distance
     */
    public boolean isFullPositionReached() {
        return isWithinRange(getLeftPosition(), EXTEND_DISTANCE, 5000) &&
                isWithinRange(getRightPosition(), EXTEND_DISTANCE, 5000);
    }

    /**
     * check if master talon motor has reached the partial intended distance
     * @return returns true if master talon has the partial reached intended distance
     */
    public boolean isPartialPositionReached() {
        return isWithinRange(getLeftPosition(), PARTIAL_DISTANCE, 5000) &&
                isWithinRange(getRightPosition(), PARTIAL_DISTANCE, 5000);
    }

    /**
     * check if master talon motor or follower talon motor has reached or exceeded current threshold (in Amps)
     * @return returns true if either talon or follower talon motor has reached or exceeded current threshold
     */
    public boolean isCurrentSpiking() {
        return leftTalonMotor.getSupplyCurrent() >= CURRENT_THRESHOLD || rightTalonMotor.getSupplyCurrent() >= CURRENT_THRESHOLD;
    }

    public boolean isLeftHangerCurrentSpiking(){
        return leftTalonMotor.getSupplyCurrent() >= CURRENT_THRESHOLD;
    }

    public boolean isLeftHangerCurrentSpiking(double threshold){
        return leftTalonMotor.getSupplyCurrent() >= threshold;
    }

    public boolean isRightHangerCurrentSpiking(){
        return rightTalonMotor.getSupplyCurrent() >= CURRENT_THRESHOLD;
    }

    public boolean isRightHangerCurrentSpiking(double threshold){
        return rightTalonMotor.getSupplyCurrent() >= threshold;
    }

    public void stopLeftMotor(){
        leftTalonMotor.set(ControlMode.PercentOutput, 0);
    }

    public void stopRightMotor(){
        rightTalonMotor.set(ControlMode.PercentOutput, 0);
    }

    public void stopMotors() {
        // Stop Motors together, by Following
        rightTalonMotor.follow(leftTalonMotor);

        leftTalonMotor.neutralOutput();
    }

    public void zeroLeftMotor(){
        leftTalonMotor.setSelectedSensorPosition(0);
    }

    public void zeroRightMotor(){
        rightTalonMotor.setSelectedSensorPosition(0);
    }

    private boolean isWithinRange(double value, double setpoint, double delta){
        return Math.abs(setpoint-value) <= delta;
    }
}
