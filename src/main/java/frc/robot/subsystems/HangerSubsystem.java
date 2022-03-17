package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
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

    private final TalonFX leftMasterTalonMotor;
    private final TalonFX rightFollowerTalonMotor;
    private final DoubleSolenoid hangerSolenoid;

    DigitalInput bottomLimitSwitch = new DigitalInput(HANGER_LIMITSWITCH_CHANNEL);

    public HangerSubsystem() {
        leftMasterTalonMotor = TalonFXFactory.createTalonFX(
                HANGER_LEFT_MASTER_TALON_ID,
                MASTER_CONFIG,
                MANI_CAN_BUS
        );

        rightFollowerTalonMotor = TalonFXFactory.createFollowerTalonFX(
                HANGER_RIGHT_FOLLOWER_TALON_ID,
                leftMasterTalonMotor,
                FOLLOWER_CONFIG,
                MANI_CAN_BUS
        );

        hangerSolenoid = new DoubleSolenoid(PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, HANGER_SOLENOID_FORWARD, HANGER_SOLENOID_BACKWARD);

        logger.info("Hanger Initialized");

        pneumaticUpright();
    }

    public void extend() {
        logger.info("Extending");
        double distance = EXTEND_DISTANCE;
        leftMasterTalonMotor.set(ControlMode.Position, distance);
    }

    public void extendContinuously() {
        logger.info("Extending Continously");
        rightFollowerTalonMotor.set(TalonFXControlMode.PercentOutput, 0.15);
    }

    public void retractContinuously() {

        logger.info("Retracting Continuously");
        leftMasterTalonMotor.set(ControlMode.PercentOutput, -1 * HANGER_ZEROING_PERCENT_SPEED);
    }

    public void retractLeftContinuouslyToZero(){
        logger.info("Retracting Left Continuously for zero");

        // Stop Following if Following
        if (rightFollowerTalonMotor.getControlMode() == ControlMode.Follower)
            rightFollowerTalonMotor.set(ControlMode.PercentOutput, 0);

        leftMasterTalonMotor.set(ControlMode.PercentOutput, -1 * HANGER_ZEROING_PERCENT_SPEED);
    }

    public void retractRightContinuouslyToZero(){
        logger.info("Retracting Right Continuously for zero");
        rightFollowerTalonMotor.set(ControlMode.PercentOutput, -1 * HANGER_ZEROING_PERCENT_SPEED);
    }

    public void retractToHang(){
        logger.info("Retracting to Hang");
        leftMasterTalonMotor.set(TalonFXControlMode.PercentOutput, -1 * HANGER_RETRACT_PERCENT_SPEED);
    }

    public void extendPartial() {
        rightFollowerTalonMotor.follow(leftMasterTalonMotor);
        logger.info("Extending Partially");
        double distance = PARTIAL_DISTANCE ;
        leftMasterTalonMotor.set(ControlMode.Position, distance);
    }

    public void pneumaticUpright() {
        hangerSolenoid.set(kForward);
    }

    public void pneumaticSlant() {
        hangerSolenoid.set(kReverse);
    }

    public void adjustRetract() {

        double distance = ADJUSTMENT_RETRACT_DISTANCE;
        leftMasterTalonMotor.set(ControlMode.Position, distance);
    }

    public void zeroHanger(){
        leftMasterTalonMotor.getSensorCollection().setIntegratedSensorPosition(0,0);
    }

    /**
     * check if the bottom limit switch is triggered
     * @return returns if the bottom limit switch is triggered
     */
    public boolean hasReachedBottom() {
        return bottomLimitSwitch.get();
    }

    /**
     * get the position of master talon in rotations of spool
     * @return returns position of master talon in rotations of spool
     */
    public double getPosition() {
        return (leftMasterTalonMotor.getSelectedSensorPosition());
    }

    /**
     * check if master talon motor has reached the full intended distance
     * @return returns true if master talon has the full reached intended distance
     */
    public boolean isFullPositionReached() {
        return getPosition() >= EXTEND_DISTANCE;
    }

    /**
     * check if master talon motor has reached the partial intended distance
     * @return returns true if master talon has the partial reached intended distance
     */
    public boolean isPartialPositionReached() {
        return getPosition() >= PARTIAL_DISTANCE;
    }

    /**
     * check if master talon motor or follower talon motor has reached or exceeded current threshold (in Amps)
     * @return returns true if either talon or follower talon motor has reached or exceeded current threshold
     */
    public boolean isCurrentSpiking() {
        return leftMasterTalonMotor.getSupplyCurrent() >= CURRENT_THRESHOLD || rightFollowerTalonMotor.getSupplyCurrent() >= CURRENT_THRESHOLD;
    }

    public boolean isLeftHangerCurrentSpiking(){
        return leftMasterTalonMotor.getSupplyCurrent() >= CURRENT_THRESHOLD;
    }

    public boolean isRightHangerCurrentSpiking(){
        return rightFollowerTalonMotor.getSupplyCurrent() >= CURRENT_THRESHOLD;
    }

    public void stopLeftMotor(){
        leftMasterTalonMotor.set(ControlMode.PercentOutput, 0);
    }

    public void stopRightMotor(){
        rightFollowerTalonMotor.set(ControlMode.PercentOutput, 0);
    }

    public void stopMotors() {
        // Stop Motors together, by Following
        rightFollowerTalonMotor.follow(leftMasterTalonMotor);

        leftMasterTalonMotor.neutralOutput();
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Left Supply",leftMasterTalonMotor.getSupplyCurrent());
        SmartDashboard.putNumber("Right Supply", rightFollowerTalonMotor.getSupplyCurrent());
    }
}
