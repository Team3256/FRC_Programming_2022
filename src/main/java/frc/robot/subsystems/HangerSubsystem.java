package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonFXFactory;
import frc.robot.helper.logging.RobotLogger;

import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
import static frc.robot.Constants.HangerConstants.*;
import static frc.robot.Constants.IDConstants.*;

public class HangerSubsystem extends SubsystemBase {
    private static final RobotLogger logger = new RobotLogger(HangerSubsystem.class.getCanonicalName());

    private final TalonFX masterTalonMotor;
    private final TalonFX followerTalonMotor;
    private final DoubleSolenoid hangerSolenoid;

    DigitalInput bottomLimitSwitch = new DigitalInput(HANGER_LIMITSWITCH_CHANNEL);

    public HangerSubsystem() {
        masterTalonMotor = TalonFXFactory.createTalonFX(
                HANGER_LEFT_MASTER_TALON_ID,
                MASTER_CONFIG,
                ROBORIO_CAN_BUS
        );

        followerTalonMotor = TalonFXFactory.createFollowerTalonFX(
                HANGER_RIGHT_FOLLOWER_TALON_ID,
                HANGER_LEFT_MASTER_TALON_ID,
                FOLLOWER_CONFIG,
                ROBORIO_CAN_BUS
        );

        hangerSolenoid = new DoubleSolenoid(PNEUMATICS_HUB_ID, PneumaticsModuleType.REVPH, HANGER_SOLENOID_FORWARD, HANGER_SOLENOID_BACKWARD);

        logger.info("Hanger Initialized");
    }

    public void extend() {
        logger.info("Extending");
        double distance = EXTEND_DISTANCE * 2048 * GEAR_RATIO;
        masterTalonMotor.set(ControlMode.Position, distance);
    }

    public void retractContinuously() {
        logger.info("Retracting Continuously");
        masterTalonMotor.set(ControlMode.PercentOutput, -1 * RETRACT_PERCENT_SPEED);
    }

    public void extendPartial() {
        logger.info("Extending Partially");
        double distance = PARTIAL_DISTANCE * 2048 * GEAR_RATIO;
        masterTalonMotor.set(ControlMode.Position, distance);
    }

    public void pneumaticUpright() {
        hangerSolenoid.set(kForward);
    }

    public void pneumaticSlant() {
        hangerSolenoid.set(kReverse);
    }

    public void adjustRetract() {
        // Reset Position, because we are at limit switch
        masterTalonMotor.getSensorCollection().setIntegratedSensorPosition(0,0);

        double distance = ADJUSTMENT_RETRACT_DISTANCE * 2048 * GEAR_RATIO;
        masterTalonMotor.set(ControlMode.Position, distance);
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
        return (masterTalonMotor.getSelectedSensorPosition()/2048)/GEAR_RATIO;
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

    public void stopMotors() {
        masterTalonMotor.neutralOutput();
    }

}
