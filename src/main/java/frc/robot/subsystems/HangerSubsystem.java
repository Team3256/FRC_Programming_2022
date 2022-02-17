package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.hardware.TalonFXFactory;
import static frc.robot.hardware.TalonFXFactory.Configuration;

import java.util.logging.Logger;

import static frc.robot.Constants.HangerConstants.*;
import static frc.robot.Constants.IDConstants.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
public class HangerSubsystem extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(HangerSubsystem.class.getCanonicalName());

    private final TalonFX masterTalonMotor;
    private final TalonFX followerTalonMotor;
    private final DoubleSolenoid leftSolenoid;
    private final DoubleSolenoid rightSolenoid;
    private final DoubleSolenoid leftAirBrake;
    private final DoubleSolenoid rightAirBrake;
    DigitalInput bottomLimitSwitch = new DigitalInput(LIMIT_SWITCH_CHANNEL);

    public HangerSubsystem() {
        Configuration MASTER_CONFIG = new Configuration();
        MASTER_CONFIG.NEUTRAL_MODE = NeutralMode.Brake;
        MASTER_CONFIG.INVERT_TYPE = INVERT_MOTOR ? InvertType.InvertMotorOutput : InvertType.None;
        MASTER_CONFIG.PIDF_CONSTANTS = new Configuration.PIDF(
                HANGER_MASTER_TALON_PID_P,
                HANGER_MASTER_TALON_PID_I,
                HANGER_MASTER_TALON_PID_D,
                HANGER_MASTER_TALON_PID_F
        );

        Configuration FOLLOWER_CONFIG = Configuration.clone(MASTER_CONFIG);
        FOLLOWER_CONFIG.INVERT_TYPE = InvertType.OpposeMaster;

        masterTalonMotor = TalonFXFactory.createTalonFX(
                HANGER_MASTER_TALON_ID,
                MASTER_CONFIG
        );
        followerTalonMotor = TalonFXFactory.createFollowerTalonFX(HANGER_FOLLOWER_TALON_ID,
                HANGER_MASTER_TALON_ID,
                FOLLOWER_CONFIG
        );

        leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, HANGER_SOLENOID_LEFT_FORWARD, HANGER_SOLENOID_LEFT_BACKWARD);
        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, HANGER_SOLENOID_RIGHT_FORWARD, HANGER_SOLENOID_RIGHT_BACKWARD);
        leftAirBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, HANGER_SOLENOID_LEFT_AIRBRAKE_FORWARD, HANGER_SOLENOID_LEFT_AIRBRAKE_BACKWARD);
        rightAirBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, HANGER_SOLENOID_RIGHT_AIRBRAKE_FORWARD, HANGER_SOLENOID_RIGHT_AIRBRAKE_BACKWARD);
        engageAirBrake();

        logger.info("Hanger Initialized");
    }
    public void engageAirBrake() {
        logger.info("Engaging Airbrake");
        leftAirBrake.set(kForward);
        rightAirBrake.set(kForward);
        //TODO: CHECK THIS
    }

    public void disengageAirBrake() {
        logger.info("Disengaging Airbrake");
        leftAirBrake.set(kReverse);
        rightAirBrake.set(kReverse);
        //TODO: CHECK THIS
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
        leftSolenoid.set(kForward);
        rightSolenoid.set(kForward);
    }

    public void pneumaticSlant() {
        leftSolenoid.set(kReverse);
        rightSolenoid.set(kReverse);
    }

    /**
     * check if the bottom limit switch is triggered
     * @return returns if the bottom limit switch is triggered
     */
    public boolean hasReachedBottom() {
        if (bottomLimitSwitch.get()) {
            return true;
        }
        return false;
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
        masterTalonMotor.set(ControlMode.PercentOutput, 0);
    }

}
