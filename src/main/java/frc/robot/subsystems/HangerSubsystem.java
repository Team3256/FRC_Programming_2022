package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.util.logging.Logger;

import static frc.robot.Constants.HangerConstants.*;
import static frc.robot.Constants.IDConstants.*;
import static edu.wpi.first.wpilibj.DoubleSolenoid.Value.*;
public class HangerSubsystem extends SubsystemBase {
    private static final Logger logger = Logger.getLogger(IntakeSubsystem.class.getCanonicalName());

    private final TalonFX masterTalonMotor;
    private final TalonFX followerTalonMotor;
    private final DoubleSolenoid leftSolenoid;
    private final DoubleSolenoid rightSolenoid;
    private final DoubleSolenoid leftAirBrake;
    private final DoubleSolenoid rightAirBrake;
    DigitalInput bottomLimitSwitch = new DigitalInput(LIMIT_SWITCH_CHANNEL);



    public HangerSubsystem() {
        masterTalonMotor = new TalonFX(HANGER_MASTER_TALON_ID);
        followerTalonMotor = new TalonFX(HANGER_FOLLOWER_TALON_ID);

        masterTalonMotor.config_kP(0, HANGER_MASTER_TALON_PID_P); //TODO: change slotIdx if required
        masterTalonMotor.config_kI(0, HANGER_MASTER_TALON_PID_I); //TODO: change slotIdx if required
        masterTalonMotor.config_kD(0, HANGER_MASTER_TALON_PID_D); //TODO: change slotIdx if required
        masterTalonMotor.config_kF(0, HANGER_MASTER_TALON_PID_F); //TODO: change slotIdx if required

        masterTalonMotor.setInverted(INVERT_MOTOR);

        followerTalonMotor.follow(masterTalonMotor);
        followerTalonMotor.setInverted(InvertType.OpposeMaster);

        masterTalonMotor.setNeutralMode(NeutralMode.Brake);
        followerTalonMotor.setNeutralMode(NeutralMode.Brake);

        leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, HANGER_SOLENOID_LEFT_FORWARD, HANGER_SOLENOID_LEFT_BACKWARD);
        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, HANGER_SOLENOID_RIGHT_FORWARD, HANGER_SOLENOID_RIGHT_BACKWARD);
        leftAirBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, HANGER_SOLENOID_LEFT_AIRBRAKE_FORWARD, HANGER_SOLENOID_LEFT_AIRBRAKE_BACKWARD);
        rightAirBrake = new DoubleSolenoid(PneumaticsModuleType.REVPH, HANGER_SOLENOID_RIGHT_AIRBRAKE_FORWARD, HANGER_SOLENOID_RIGHT_AIRBRAKE_BACKWARD);
        engageAirBrake();
   }
    public void engageAirBrake() {
        leftAirBrake.set(kForward);
        rightAirBrake.set(kForward);
        //TODO: CHECK THIS
    }

    public void disengageAirBrake() {
        leftAirBrake.set(kReverse);
        rightAirBrake.set(kReverse);
        //TODO: CHECK THIS
    }

    public void extend() {
        double distance = EXTEND_DISTANCE * 2048 * GEAR_RATIO;
        masterTalonMotor.set(ControlMode.Position, distance);
    }

    public void retractContinuously() {
        masterTalonMotor.set(ControlMode.PercentOutput, -1 * RETRACT_PERCENT_SPEED);
    }

    public void extendPartial() {
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
        logger.info("Hanger stopped");
        masterTalonMotor.set(ControlMode.PercentOutput, 0);
    }

}